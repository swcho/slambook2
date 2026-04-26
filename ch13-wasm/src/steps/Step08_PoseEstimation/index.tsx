import { useEffect, useMemo, useRef, useState } from 'react';
import { useQuery } from '@tanstack/react-query';
import { STEPS } from '../index';
import { StepLayout } from '../../components/StepLayout';
import type { VerifyItem } from '../../components/VerifyGate';
import { loadKittiFrame, parseKittiCalib } from '../../lib/kitti';
import { imageDataToGray, loadFeaturesWasm, type FeaturesModule } from '../../wasm/features';
import {
  loadTriangulationWasm,
  makeK,
  makeRectifiedT,
  type TriangulationModule,
} from '../../wasm/triangulation';
import {
  loadPnPWasm,
  makeKMatrix,
  identityInit,
  type PnPModule,
  type PnPResult,
} from '../../wasm/pnp';

// Step 8 mirrors ch13/src/frontend.cpp::EstimateCurrentPose. Pipeline:
//   1. detect features on frame[i-1] left
//   2. stereo-LK to frame[i-1] right + Linear-SVD triangulation → 3D points in
//      the left camera (= world) frame
//   3. LK from frame[i-1] left → frame[i] left → frame[i] 2D observations
//   4. pose-only PnP (3D ↔ 2D) with the 4-round outlier loop
// This recovers the relative pose of frame[i] in the previous frame's camera
// frame. PLAN §3 Step 8 calls for chi² / iterations / Huber δ / RobustKernel
// drop-time controls plus per-round outlier visualisation; we implement all
// four and surface the round masks as a slider-driven animation.

const DATASET_DIR = '/datasets/kitti05-mini';
const FRAME_COUNT = 5;
// PLAN §3 Step 8: inlier ratio > 70% verification gate.
const INLIER_RATIO_GATE = 0.7;

interface Step8Params {
  rounds: number;
  iterPerRound: number;
  chi2Threshold: number;
  huberDelta: number;
  removeRobustAfterRound: number;
  useRobustKernel: boolean;
  maxFeatures: number;
}

const DEFAULT_PARAMS: Step8Params = {
  rounds: 4, // ch13 EstimateCurrentPose loop count
  iterPerRound: 10, // ch13 optimizer.optimize(10)
  chi2Threshold: 5.991, // 95% chi² threshold for 2 DOF
  huberDelta: Math.sqrt(5.991),
  removeRobustAfterRound: 2, // ch13: drops kernel for last 2 rounds
  useRobustKernel: true,
  maxFeatures: 200,
};

interface PoseRunResult {
  /** Final 4×4 row-major Tcw — frame[i] camera in frame[i-1] world. */
  Tcw: Float64Array;
  /** Per-pair inlier flags after final round (length = pairCount). */
  finalInlierMask: Uint8Array;
  /** [round * pairCount + i] → 1 inlier / 0 outlier. */
  roundInlierMasks: Uint8Array;
  roundInlierCount: Int32Array;
  roundIters: Int32Array;
  finalChi2: number;
  totalInliers: number;
  pairCount: number;
  rounds: number;
  /** length pairCount * 2 — 2D obs in frame[i] (used for overlay). */
  obs2d: Float64Array;
  detectMs: number;
  stereoLkMs: number;
  triMs: number;
  trackLkMs: number;
  pnpMs: number;
  /** Recovered relative motion summary (axis-angle + translation). */
  rotationDeg: number;
  translation: [number, number, number];
}

export function Step08PoseEstimation() {
  const step = STEPS.find((s) => s.id === 8)!;
  const [currIndex, setCurrIndex] = useState(1);
  const [params, setParams] = useState<Step8Params>(DEFAULT_PARAMS);
  const [roundView, setRoundView] = useState(0);

  const downsample = 0.5;
  const cameras = useQuery({
    queryKey: ['kitti', 'calib', DATASET_DIR, downsample],
    queryFn: async () => {
      const res = await fetch(`${DATASET_DIR}/calib.txt`);
      if (!res.ok) throw new Error(`calib.txt: HTTP ${res.status}`);
      return parseKittiCalib(await res.text(), downsample);
    },
  });
  const prevFrame = useQuery({
    queryKey: ['kitti', 'frame', DATASET_DIR, currIndex - 1, downsample],
    queryFn: () => loadKittiFrame(DATASET_DIR, currIndex - 1, downsample),
    enabled: currIndex >= 1,
  });
  const currFrame = useQuery({
    queryKey: ['kitti', 'frame', DATASET_DIR, currIndex, downsample],
    queryFn: () => loadKittiFrame(DATASET_DIR, currIndex, downsample),
  });
  const features = useQuery<FeaturesModule>({
    queryKey: ['wasm', 'features', 'baseline'],
    queryFn: () => loadFeaturesWasm('baseline'),
  });
  const tri = useQuery<TriangulationModule>({
    queryKey: ['wasm', 'triangulation', 'baseline'],
    queryFn: () => loadTriangulationWasm('baseline'),
  });
  const pnp = useQuery<PnPModule>({
    queryKey: ['wasm', 'pnp', 'baseline'],
    queryFn: () => loadPnPWasm('baseline'),
  });

  const grayPrevLeft = useMemo(
    () => (prevFrame.data ? imageDataToGray(prevFrame.data.left) : null),
    [prevFrame.data],
  );
  const grayPrevRight = useMemo(
    () => (prevFrame.data ? imageDataToGray(prevFrame.data.right) : null),
    [prevFrame.data],
  );
  const grayCurrLeft = useMemo(
    () => (currFrame.data ? imageDataToGray(currFrame.data.left) : null),
    [currFrame.data],
  );

  const left = cameras.data?.find((c) => c.id === 0) ?? null;
  const right = cameras.data?.find((c) => c.id === 1) ?? null;

  const result = useMemo<PoseRunResult | null>(() => {
    if (
      !features.data || !tri.data || !pnp.data ||
      !prevFrame.data || !currFrame.data ||
      !grayPrevLeft || !grayPrevRight || !grayCurrLeft ||
      !left || !right
    ) {
      return null;
    }
    const w = prevFrame.data.width;
    const h = prevFrame.data.height;

    // Step 3 — detect on previous frame left
    const t0 = performance.now();
    const seeds = features.data.detect(grayPrevLeft, w, h, 'GFTT', {
      maxFeatures: params.maxFeatures,
      qualityLevel: 0.01,
      minDistance: 20,
    });
    const detectMs = performance.now() - t0;
    if (seeds.length === 0) return null;

    // Step 4 — stereo LK prev-left → prev-right
    const t1 = performance.now();
    const stereo = features.data.track(grayPrevLeft, grayPrevRight, w, h, seeds, {
      winSize: 11,
      maxLevel: 3,
      maxIter: 30,
      eps: 0.01,
    });
    const stereoLkMs = performance.now() - t1;

    // Step 5 — triangulate stereo pairs
    const kL = makeK(left.fx, left.fy, left.cx, left.cy);
    const kR = makeK(right.fx, right.fy, right.cx, right.cy);
    const tL = makeRectifiedT([0, 0, 0]);
    const tR = makeRectifiedT(right.t);
    const t2 = performance.now();
    const triPts = tri.data.triangulate(seeds, stereo, kL, tL, kR, tR, {
      algo: 'LinearSVD',
      qualityThreshold: 0.01,
    });
    const triMs = performance.now() - t2;

    // Step 7 — temporal LK prev-left → curr-left
    const t3 = performance.now();
    const tracked = features.data.track(grayPrevLeft, grayCurrLeft, w, h, seeds, {
      winSize: 11,
      maxLevel: 3,
      maxIter: 30,
      eps: 0.01,
    });
    const trackLkMs = performance.now() - t3;

    // Build PnP input pairs: keep only points where stereo triangulation
    // succeeded AND temporal LK succeeded.
    const total = seeds.length / 3;
    const pts3: number[] = [];
    const pts2: number[] = [];
    for (let i = 0; i < total; i++) {
      const triOk = triPts[i * 5 + 4] > 0.5;
      const lkOk = tracked[i * 3 + 2] > 0.5;
      if (!triOk || !lkOk) continue;
      pts3.push(triPts[i * 5 + 0], triPts[i * 5 + 1], triPts[i * 5 + 2]);
      pts2.push(tracked[i * 3 + 0], tracked[i * 3 + 1]);
    }
    const pairCount = pts3.length / 3;
    if (pairCount < 4) return null;

    // Pose-only PnP. We start from identity since the synthetic fixture has
    // tiny inter-frame motion (~7 px shift @ 0.5×). For real KITTI we'd seed
    // with a constant-velocity prior — same shape, different init6.
    const K = makeKMatrix(left.fx, left.fy, left.cx, left.cy);
    const t4 = performance.now();
    const res: PnPResult = pnp.data.estimatePose(
      new Float64Array(pts3),
      new Float64Array(pts2),
      K,
      identityInit(),
      {
        rounds: params.rounds,
        iterPerRound: params.iterPerRound,
        chi2Threshold: params.chi2Threshold,
        huberDelta: params.huberDelta,
        removeRobustAfterRound: params.removeRobustAfterRound,
        useRobustKernel: params.useRobustKernel,
      },
    );
    const pnpMs = performance.now() - t4;

    // Decompose Tcw for human-readable display.
    const T = res.Tcw_row_major;
    const trace = T[0] + T[5] + T[10];
    const cosTheta = Math.max(-1, Math.min(1, (trace - 1) / 2));
    const rotationRad = Math.acos(cosTheta);
    const translation: [number, number, number] = [T[3], T[7], T[11]];

    return {
      Tcw: T,
      finalInlierMask: res.finalInlierMask,
      roundInlierMasks: res.roundInlierMasks,
      roundInlierCount: res.roundInlierCount,
      roundIters: res.roundIters,
      finalChi2: res.finalChi2,
      totalInliers: res.totalInliers,
      pairCount,
      rounds: res.rounds,
      obs2d: new Float64Array(pts2),
      detectMs,
      stereoLkMs,
      triMs,
      trackLkMs,
      pnpMs,
      rotationDeg: (rotationRad * 180) / Math.PI,
      translation,
    };
  }, [
    features.data, tri.data, pnp.data,
    prevFrame.data, currFrame.data,
    grayPrevLeft, grayPrevRight, grayCurrLeft,
    left, right, params,
  ]);

  // Clamp roundView when params.rounds shrinks.
  useEffect(() => {
    if (result && roundView >= result.rounds) setRoundView(result.rounds - 1);
  }, [result, roundView]);

  const verifyItems: VerifyItem[] = useMemo(() => {
    const items: VerifyItem[] = [];
    items.push({
      id: 'wasm-loaded',
      label: 'features + triangulation + pnp WASM 모듈 로드',
      pass: !!features.data && !!tri.data && !!pnp.data,
    });
    items.push({
      id: 'frames-loaded',
      label: 'prev (L+R) + curr (L) + calib 로드',
      pass: !!prevFrame.data && !!currFrame.data && !!cameras.data,
      detail: prevFrame.data && currFrame.data ? `frames #${currIndex - 1} → #${currIndex}` : undefined,
    });
    if (result) {
      items.push({
        id: 'pair-count',
        label: '3D-2D 페어 수 ≥ 30 (PnP 입력)',
        pass: result.pairCount >= 30,
        detail: `${result.pairCount} pairs`,
      });
      const ratio = result.pairCount > 0 ? result.totalInliers / result.pairCount : 0;
      items.push({
        id: 'inlier-ratio',
        label: `inlier 비율 > ${(INLIER_RATIO_GATE * 100).toFixed(0)}% (PLAN §3 Step 8)`,
        pass: ratio > INLIER_RATIO_GATE,
        detail: `${result.totalInliers} / ${result.pairCount} (${(ratio * 100).toFixed(1)}%)`,
      });
      items.push({
        id: 'chi2-finite',
        label: 'final chi² 유한값',
        pass: Number.isFinite(result.finalChi2),
        detail: Number.isFinite(result.finalChi2) ? result.finalChi2.toExponential(2) : undefined,
      });
    } else {
      items.push({ id: 'pair-count', label: '3D-2D 페어 수 ≥ 30', pass: false, detail: '계산 대기' });
      items.push({ id: 'inlier-ratio', label: 'inlier 비율 > 70%', pass: false });
      items.push({ id: 'chi2-finite', label: 'final chi² 유한값', pass: false });
    }
    return items;
  }, [features.data, tri.data, pnp.data, prevFrame.data, currFrame.data, cameras.data, result, currIndex]);

  return (
    <StepLayout
      step={step}
      paramPanel={
        <ParamPanel
          currIndex={currIndex}
          setCurrIndex={setCurrIndex}
          params={params}
          setParams={setParams}
        />
      }
      input={<InputView prev={prevFrame.data ?? null} curr={currFrame.data ?? null} />}
      output={
        <OutputView
          curr={currFrame.data ?? null}
          result={result}
          roundView={roundView}
          setRoundView={setRoundView}
        />
      }
      verifyItems={verifyItems}
    />
  );
}

function ParamPanel({
  currIndex,
  setCurrIndex,
  params,
  setParams,
}: {
  currIndex: number;
  setCurrIndex: (n: number) => void;
  params: Step8Params;
  setParams: (next: Step8Params) => void;
}) {
  const update = <K extends keyof Step8Params>(key: K, value: Step8Params[K]) =>
    setParams({ ...params, [key]: value });
  return (
    <section style={paramPanelStyle}>
      <h3 style={sectionTitle}>PnP algorithm</h3>
      <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6 }}>
        <button type="button" style={{ ...algoBtn, background: '#2a3d5c' }}>
          g2o LM (4-round)
        </button>
        <span
          title="Available once OpenCV calib3d is added to the BUILD_LIST (Phase E+ deferred)."
          style={algoBtnDisabled}
        >
          cv::solvePnPRansac (Step 8+)
        </span>
        <span style={algoBtnDisabled} title="Closed-form alternatives — Phase E+">
          EPnP / DLS PnP
        </span>
      </div>

      <Slider label="curr frame" value={currIndex} min={1} max={FRAME_COUNT - 1} step={1} onChange={setCurrIndex} format={(n) => `#${n - 1} → #${n}`} />
      <Slider label="maxFeatures" value={params.maxFeatures} min={20} max={500} step={10} onChange={(n) => update('maxFeatures', n)} format={(n) => String(n)} />

      <h3 style={sectionTitle}>Outer loop (rounds × iters)</h3>
      <Slider label="rounds" value={params.rounds} min={1} max={6} step={1} onChange={(n) => update('rounds', n)} format={(n) => String(n)} />
      <Slider label="iterPerRound" value={params.iterPerRound} min={1} max={20} step={1} onChange={(n) => update('iterPerRound', n)} format={(n) => String(n)} />

      <h3 style={sectionTitle}>Outlier rejection</h3>
      <Slider
        label="chi² threshold"
        value={params.chi2Threshold}
        min={0.5}
        max={20}
        step={0.001}
        onChange={(n) => update('chi2Threshold', n)}
        format={(n) => n.toFixed(3)}
      />
      <Slider
        label="Huber δ"
        value={params.huberDelta}
        min={0.1}
        max={10}
        step={0.01}
        onChange={(n) => update('huberDelta', n)}
        format={(n) => n.toFixed(2)}
      />
      <Slider
        label="drop kernel after round"
        value={params.removeRobustAfterRound}
        min={0}
        max={Math.max(1, params.rounds)}
        step={1}
        onChange={(n) => update('removeRobustAfterRound', n)}
        format={(n) => String(n)}
      />
      <label style={{ ...labelStyle, flexDirection: 'row', alignItems: 'center', gap: 6 }}>
        <input
          type="checkbox"
          checked={params.useRobustKernel}
          onChange={(e) => update('useRobustKernel', e.target.checked)}
        />
        <span>RobustKernel (Huber)</span>
      </label>

      <button
        type="button"
        onClick={() => setParams(DEFAULT_PARAMS)}
        style={resetBtn}
      >
        reset to ch13 defaults (4×10, χ²=5.991, drop=2)
      </button>
      <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
        ch13 EstimateCurrentPose: 4 라운드 × 10 LM iter, chi² &gt; 5.991 → outlier,
        round ≥ 2부터 RobustKernel 제거. PLAN §3 Step 8.
      </div>
    </section>
  );
}

function Slider({
  label, value, min, max, step, onChange, format,
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  onChange: (n: number) => void;
  format: (n: number) => string;
}) {
  return (
    <label style={labelStyle}>
      <span style={{ display: 'flex', justifyContent: 'space-between' }}>
        <span>{label}</span>
        <strong>{format(value)}</strong>
      </span>
      <input type="range" min={min} max={max} step={step} value={value} onChange={(e) => onChange(Number(e.target.value))} />
    </label>
  );
}

function InputView({
  prev, curr,
}: {
  prev: { width: number; height: number; left: ImageData; right: ImageData; index: number } | null;
  curr: { width: number; height: number; left: ImageData; index: number } | null;
}) {
  const prevLeftRef = useRef<HTMLCanvasElement | null>(null);
  const prevRightRef = useRef<HTMLCanvasElement | null>(null);
  const currRef = useRef<HTMLCanvasElement | null>(null);
  useEffect(() => {
    if (prev && prevLeftRef.current) {
      prevLeftRef.current.width = prev.width;
      prevLeftRef.current.height = prev.height;
      prevLeftRef.current.getContext('2d')?.putImageData(prev.left, 0, 0);
    }
    if (prev && prevRightRef.current) {
      prevRightRef.current.width = prev.width;
      prevRightRef.current.height = prev.height;
      prevRightRef.current.getContext('2d')?.putImageData(prev.right, 0, 0);
    }
    if (curr && currRef.current) {
      currRef.current.width = curr.width;
      currRef.current.height = curr.height;
      currRef.current.getContext('2d')?.putImageData(curr.left, 0, 0);
    }
  }, [prev, curr]);
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
      <div style={{ fontSize: 12, color: '#888' }}>
        {prev && curr ? `prev #${prev.index} (L+R, 3D source) → curr #${curr.index} (L, 2D obs)` : '—'}
      </div>
      <figure style={figStyle}>
        <figcaption style={figCaption}>prev left (Step 3 detect)</figcaption>
        <canvas ref={prevLeftRef} style={canvasStyle} />
      </figure>
      <figure style={figStyle}>
        <figcaption style={figCaption}>prev right (Step 4 stereo LK)</figcaption>
        <canvas ref={prevRightRef} style={canvasStyle} />
      </figure>
      <figure style={figStyle}>
        <figcaption style={figCaption}>curr left (Step 7 temporal LK + PnP obs)</figcaption>
        <canvas ref={currRef} style={canvasStyle} />
      </figure>
    </div>
  );
}

function OutputView({
  curr, result, roundView, setRoundView,
}: {
  curr: { width: number; height: number; left: ImageData } | null;
  result: PoseRunResult | null;
  roundView: number;
  setRoundView: (n: number) => void;
}) {
  const ref = useRef<HTMLCanvasElement | null>(null);
  useEffect(() => {
    if (!curr || !ref.current) return;
    const canvas = ref.current;
    canvas.width = curr.width;
    canvas.height = curr.height;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.putImageData(curr.left, 0, 0);
    if (!result) return;
    // Render the per-round inlier mask the user picked.
    const round = Math.max(0, Math.min(roundView, result.rounds - 1));
    const N = result.pairCount;
    for (let i = 0; i < N; i++) {
      const x = result.obs2d[i * 2 + 0];
      const y = result.obs2d[i * 2 + 1];
      const ok = result.roundInlierMasks[round * N + i] === 1;
      ctx.fillStyle = ok ? 'rgba(60, 220, 120, 0.85)' : 'rgba(255, 90, 90, 0.85)';
      ctx.beginPath();
      ctx.arc(x, y, 3, 0, Math.PI * 2);
      ctx.fill();
    }
  }, [curr, result, roundView]);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      {result ? (
        <div style={{ fontSize: 12, color: '#aaa', lineHeight: 1.5 }}>
          inliers (final) <strong>{result.totalInliers}</strong> / {result.pairCount} ·{' '}
          chi² <strong>{result.finalChi2.toExponential(2)}</strong>{' '}
          · rotation <strong>{result.rotationDeg.toFixed(2)}°</strong>{' '}
          · t = [{result.translation.map((v) => v.toFixed(3)).join(', ')}]
          <br />
          detect {result.detectMs.toFixed(1)} ms · stereoLK {result.stereoLkMs.toFixed(1)} ms ·
          {' '}tri {result.triMs.toFixed(1)} ms · trackLK {result.trackLkMs.toFixed(1)} ms ·
          {' '}PnP <strong>{result.pnpMs.toFixed(1)} ms</strong>
        </div>
      ) : (
        <div style={{ fontSize: 12, color: '#666' }}>(waiting for inputs)</div>
      )}

      {result && (
        <RoundSlider
          rounds={result.rounds}
          roundIters={result.roundIters}
          inlierCount={result.roundInlierCount}
          pairCount={result.pairCount}
          roundView={roundView}
          setRoundView={setRoundView}
        />
      )}

      <canvas ref={ref} style={canvasStyle} />
      <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
        ● 점 = curr 프레임의 2D 관측. 녹색 = 라운드 끝 inlier, 빨강 = outlier (chi² &gt; 임계).
        라운드 슬라이더로 책의 4-라운드 outlier 제거 과정을 단계별로 확인.
      </div>

      {result && (
        <RoundChart
          counts={result.roundInlierCount}
          pairCount={result.pairCount}
          activeRound={roundView}
          onPickRound={setRoundView}
        />
      )}
    </div>
  );
}

function RoundSlider({
  rounds, roundIters, inlierCount, pairCount, roundView, setRoundView,
}: {
  rounds: number;
  roundIters: Int32Array;
  inlierCount: Int32Array;
  pairCount: number;
  roundView: number;
  setRoundView: (n: number) => void;
}) {
  const round = Math.max(0, Math.min(roundView, rounds - 1));
  return (
    <label style={labelStyle}>
      <span style={{ display: 'flex', justifyContent: 'space-between' }}>
        <span>round view</span>
        <strong>
          #{round + 1}/{rounds} · iters {roundIters[round] ?? 0} · inliers {inlierCount[round] ?? 0}/{pairCount}
        </strong>
      </span>
      <input
        type="range"
        min={0}
        max={rounds - 1}
        step={1}
        value={round}
        onChange={(e) => setRoundView(Number(e.target.value))}
      />
    </label>
  );
}

function RoundChart({
  counts, pairCount, activeRound, onPickRound,
}: {
  counts: Int32Array;
  pairCount: number;
  activeRound: number;
  onPickRound: (n: number) => void;
}) {
  const w = 280;
  const h = 100;
  const pad = { l: 28, r: 8, t: 8, b: 22 };
  const innerW = w - pad.l - pad.r;
  const innerH = h - pad.t - pad.b;
  const maxY = Math.max(pairCount, 1);
  const xFor = (i: number) => pad.l + (innerW * (i + 0.5)) / counts.length;
  const yFor = (v: number) => pad.t + innerH - (innerH * v) / maxY;
  const barWidth = (innerW / counts.length) * 0.6;
  return (
    <figure style={{ margin: 0 }}>
      <figcaption style={{ ...figCaption, marginBottom: 4 }}>
        라운드별 inlier 수 — 클릭하면 해당 라운드 마스크가 캔버스에 표시
      </figcaption>
      <svg
        width={w}
        height={h}
        style={{ background: '#0c0c0c', border: '1px solid #333', borderRadius: 4, cursor: 'pointer' }}
      >
        <line x1={pad.l} y1={pad.t} x2={pad.l} y2={pad.t + innerH} stroke="#444" strokeWidth={1} />
        <line x1={pad.l} y1={pad.t + innerH} x2={pad.l + innerW} y2={pad.t + innerH} stroke="#444" strokeWidth={1} />
        <text x={pad.l - 4} y={pad.t + 4} fill="#888" fontSize={9} textAnchor="end">{maxY}</text>
        <text x={pad.l - 4} y={pad.t + innerH} fill="#888" fontSize={9} textAnchor="end">0</text>
        {Array.from(counts).map((c, i) => {
          const x = xFor(i);
          const y = yFor(c);
          return (
            <g key={i} onClick={() => onPickRound(i)}>
              <rect
                x={x - barWidth / 2}
                y={y}
                width={barWidth}
                height={innerH - (y - pad.t)}
                fill={i === activeRound ? '#33aaff' : '#3a72b2'}
              />
              <text x={x} y={pad.t + innerH + 12} fill="#aaa" fontSize={9} textAnchor="middle">
                R{i + 1}
              </text>
              <text x={x} y={y - 2} fill="#ccc" fontSize={9} textAnchor="middle">
                {c}
              </text>
            </g>
          );
        })}
      </svg>
    </figure>
  );
}

const paramPanelStyle: React.CSSProperties = {
  border: '1px solid #333',
  borderRadius: 6,
  padding: 12,
  background: '#181818',
  display: 'flex',
  flexDirection: 'column',
  gap: 10,
};

const sectionTitle: React.CSSProperties = {
  margin: '0 0 4px',
  fontSize: 13,
  color: '#aaa',
  textTransform: 'uppercase',
  letterSpacing: 0.5,
};

const labelStyle: React.CSSProperties = {
  display: 'flex',
  flexDirection: 'column',
  gap: 4,
  fontSize: 13,
  color: '#ccc',
};

const algoBtn: React.CSSProperties = {
  padding: '4px 10px',
  borderRadius: 4,
  border: '1px solid #444',
  color: '#eee',
  cursor: 'pointer',
  fontSize: 13,
};

const algoBtnDisabled: React.CSSProperties = {
  ...algoBtn,
  background: '#1a1a1a',
  color: '#666',
  border: '1px dashed #444',
  cursor: 'not-allowed',
};

const resetBtn: React.CSSProperties = {
  padding: '4px 10px',
  borderRadius: 4,
  border: '1px solid #444',
  background: '#222',
  color: '#eee',
  cursor: 'pointer',
  fontSize: 12,
  alignSelf: 'flex-start',
};

const figStyle: React.CSSProperties = { margin: 0, display: 'flex', flexDirection: 'column', gap: 2 };
const figCaption: React.CSSProperties = { fontSize: 11, color: '#888' };
const canvasStyle: React.CSSProperties = {
  width: '100%',
  height: 'auto',
  border: '1px solid #333',
  background: '#000',
  imageRendering: 'pixelated',
};
