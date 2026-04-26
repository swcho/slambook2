import { useEffect, useMemo, useRef, useState } from 'react';
import { useQuery } from '@tanstack/react-query';
import { STEPS } from '../index';
import { StepLayout } from '../../components/StepLayout';
import type { VerifyItem } from '../../components/VerifyGate';
import { loadKittiFrame, parseKittiCalib } from '../../lib/kitti';
import {
  imageDataToGray,
  loadFeaturesWasm,
  type DetectorKey,
  type FeaturesModule,
} from '../../wasm/features';

// Step 7 mirrors ch13/src/frontend.cpp::TrackLastFrame: detect on the
// previous left frame, then LK from prev → curr (same camera, time delta).
// PLAN §3 Step 7 calls for an init-strategy toggle (map-projection /
// previous-position / none). Map-projection requires an estimated relative
// pose, which only becomes available once Step 8 (PnP) lands; until then we
// expose the two reachable strategies plus a velocity-prior toggle that lets
// learners observe how a temporal prior changes LK convergence.

const DATASET_DIR = '/datasets/kitti05-mini';
const FRAME_COUNT = 5;
const DETECTORS: DetectorKey[] = ['GFTT', 'Harris', 'FAST', 'ORB'];
const ITER_SWEEP = [1, 2, 3, 5, 8, 12, 20, 30] as const;

// ch13/include/myslam/frontend.h: num_features_tracking default = 50.
const NUM_FEATURES_TRACKING = 50;

type InitStrategy = 'none' | 'velocity';

interface Step7Params {
  detector: DetectorKey;
  maxFeatures: number;
  qualityLevel: number;
  minDistance: number;
  winSize: number;
  maxLevel: number;
  maxIter: number;
  eps: number;
  initStrategy: InitStrategy;
  velocityDx: number; // pixels (downsampled coords)
  velocityDy: number;
}

const DEFAULT_PARAMS: Step7Params = {
  detector: 'GFTT',
  maxFeatures: 150,
  qualityLevel: 0.01,
  minDistance: 20,
  winSize: 11, // ch13 TrackLastFrame: cv::Size(11, 11)
  maxLevel: 3,
  maxIter: 30,
  eps: 0.01,
  initStrategy: 'velocity',
  // Synthetic fixture forward shift: 14 px/frame in raw, 7 px @ 0.5× downsample.
  // Default seeds the velocity prior at the ground-truth shift so learners can
  // confirm convergence first, then dial it away to see degradation.
  velocityDx: -7,
  velocityDy: 0,
};

interface MatchSummary {
  trackedCount: number;
  totalCount: number;
  meanDx: number;
  meanDy: number;
  meanAbsDy: number;
  ms: number;
  detectMs: number;
}

interface IterSweepRow {
  maxIter: number;
  noneTracked: number;
  velocityTracked: number;
}

export function Step07FrameTracking() {
  const step = STEPS.find((s) => s.id === 7)!;
  const [currIndex, setCurrIndex] = useState(1);
  const [params, setParams] = useState<Step7Params>(DEFAULT_PARAMS);

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
  const wasm = useQuery<FeaturesModule>({
    queryKey: ['wasm', 'features', 'baseline'],
    queryFn: () => loadFeaturesWasm('baseline'),
  });

  const grayPrev = useMemo(
    () => (prevFrame.data ? imageDataToGray(prevFrame.data.left) : null),
    [prevFrame.data],
  );
  const grayCurr = useMemo(
    () => (currFrame.data ? imageDataToGray(currFrame.data.left) : null),
    [currFrame.data],
  );

  // Detect once on the prev left image (cheap, only when params change).
  const seeds = useMemo(() => {
    if (!wasm.data || !prevFrame.data || !grayPrev) return null;
    const t0 = performance.now();
    const out = wasm.data.detect(
      grayPrev,
      prevFrame.data.width,
      prevFrame.data.height,
      params.detector,
      {
        maxFeatures: params.maxFeatures,
        qualityLevel: params.qualityLevel,
        minDistance: params.minDistance,
      },
    );
    const detectMs = performance.now() - t0;
    return { pts: out, detectMs };
  }, [
    wasm.data,
    prevFrame.data,
    grayPrev,
    params.detector,
    params.maxFeatures,
    params.qualityLevel,
    params.minDistance,
  ]);

  const initialPts = useMemo(() => {
    if (!seeds || params.initStrategy !== 'velocity') return null;
    const out = new Float64Array(seeds.pts.length);
    for (let i = 0; i < seeds.pts.length; i += 3) {
      out[i + 0] = seeds.pts[i + 0] + params.velocityDx;
      out[i + 1] = seeds.pts[i + 1] + params.velocityDy;
      out[i + 2] = 1; // unused score field, mirrors detect output convention
    }
    return out;
  }, [seeds, params.initStrategy, params.velocityDx, params.velocityDy]);

  // Run LK with the active strategy at the user's maxIter setting.
  const result = useMemo(() => {
    if (!wasm.data || !currFrame.data || !grayPrev || !grayCurr || !seeds) return null;
    if (seeds.pts.length === 0) {
      const empty: MatchSummary = {
        trackedCount: 0,
        totalCount: 0,
        meanDx: NaN,
        meanDy: NaN,
        meanAbsDy: NaN,
        ms: 0,
        detectMs: seeds.detectMs,
      };
      return { tracked: new Float64Array(0), summary: empty };
    }
    const t1 = performance.now();
    const tracked = wasm.data.track(
      grayPrev,
      grayCurr,
      currFrame.data.width,
      currFrame.data.height,
      seeds.pts,
      {
        winSize: params.winSize,
        maxLevel: params.maxLevel,
        maxIter: params.maxIter,
        eps: params.eps,
        useInitialFlow: params.initStrategy === 'velocity',
        initialPts: params.initStrategy === 'velocity' && initialPts ? initialPts : undefined,
      },
    );
    const ms = performance.now() - t1;
    let trackedCount = 0;
    let dxSum = 0;
    let dySum = 0;
    let absDySum = 0;
    const total = seeds.pts.length / 3;
    for (let i = 0; i < total; i++) {
      const status = tracked[i * 3 + 2];
      if (status > 0.5) {
        trackedCount++;
        const dx = tracked[i * 3 + 0] - seeds.pts[i * 3 + 0];
        const dy = tracked[i * 3 + 1] - seeds.pts[i * 3 + 1];
        dxSum += dx;
        dySum += dy;
        absDySum += Math.abs(dy);
      }
    }
    const summary: MatchSummary = {
      trackedCount,
      totalCount: total,
      meanDx: trackedCount ? dxSum / trackedCount : NaN,
      meanDy: trackedCount ? dySum / trackedCount : NaN,
      meanAbsDy: trackedCount ? absDySum / trackedCount : NaN,
      ms,
      detectMs: seeds.detectMs,
    };
    return { tracked, summary };
  }, [wasm.data, currFrame.data, grayPrev, grayCurr, seeds, initialPts, params]);

  // Iteration sweep: for both strategies, count tracked points at each
  // maxIter cap. Educational purpose: visualise the "초기치 전략별 수렴
  // 라운드 수" comparison from PLAN §3 Step 7.
  const iterSweep = useMemo<IterSweepRow[] | null>(() => {
    if (!wasm.data || !currFrame.data || !grayPrev || !grayCurr || !seeds || seeds.pts.length === 0) {
      return null;
    }
    const total = seeds.pts.length / 3;
    const noneInit = (() => undefined)();
    const velInit = initialPts ?? buildVelocityInit(seeds.pts, params.velocityDx, params.velocityDy);
    const rows: IterSweepRow[] = [];
    for (const cap of ITER_SWEEP) {
      const lkOpts = {
        winSize: params.winSize,
        maxLevel: params.maxLevel,
        maxIter: cap,
        eps: params.eps,
      };
      const noneOut = wasm.data.track(
        grayPrev,
        grayCurr,
        currFrame.data.width,
        currFrame.data.height,
        seeds.pts,
        { ...lkOpts, useInitialFlow: false, initialPts: noneInit },
      );
      const velOut = wasm.data.track(
        grayPrev,
        grayCurr,
        currFrame.data.width,
        currFrame.data.height,
        seeds.pts,
        { ...lkOpts, useInitialFlow: true, initialPts: velInit },
      );
      rows.push({
        maxIter: cap,
        noneTracked: countTracked(noneOut, total),
        velocityTracked: countTracked(velOut, total),
      });
    }
    return rows;
  }, [
    wasm.data,
    currFrame.data,
    grayPrev,
    grayCurr,
    seeds,
    initialPts,
    params.velocityDx,
    params.velocityDy,
    params.winSize,
    params.maxLevel,
    params.eps,
  ]);

  const verifyItems: VerifyItem[] = useMemo(() => {
    const items: VerifyItem[] = [];
    items.push({
      id: 'wasm-loaded',
      label: 'features WASM 모듈 로드',
      pass: !!wasm.data,
      detail: wasm.data ? `OpenCV ${wasm.data.opencvVersion}` : wasm.error ? String(wasm.error) : '로딩 중…',
    });
    items.push({
      id: 'temporal-pair',
      label: 'prev + curr 프레임 + calib 로드',
      pass: !!prevFrame.data && !!currFrame.data && !!cameras.data,
      detail: prevFrame.data && currFrame.data ? `frames #${currIndex - 1} → #${currIndex}` : undefined,
    });
    if (result?.summary) {
      const s = result.summary;
      items.push({
        id: 'tracked-min',
        label: `tracked ≥ ${NUM_FEATURES_TRACKING} (num_features_tracking)`,
        pass: s.trackedCount >= NUM_FEATURES_TRACKING,
        detail: `${s.trackedCount} / ${s.totalCount}`,
      });
      const expectedDx = -14 * downsample; // synthetic fixture GT
      const dxOk = Number.isFinite(s.meanDx) && Math.abs(s.meanDx - expectedDx) <= 2;
      items.push({
        id: 'mean-dx-gt',
        label: `평균 dx ≈ ${expectedDx} px (합성 GT ± 2 px)`,
        pass: dxOk,
        detail: Number.isFinite(s.meanDx) ? `mean dx = ${s.meanDx.toFixed(2)} px` : undefined,
      });
      items.push({
        id: 'epipolar',
        label: '평균 |dy| ≤ 2 px (수직 잔차)',
        pass: Number.isFinite(s.meanAbsDy) && s.meanAbsDy <= 2,
        detail: Number.isFinite(s.meanAbsDy) ? `mean |dy| = ${s.meanAbsDy.toFixed(2)} px` : undefined,
      });
    } else {
      items.push({ id: 'tracked-min', label: `tracked ≥ ${NUM_FEATURES_TRACKING}`, pass: false, detail: '계산 대기' });
      items.push({ id: 'mean-dx-gt', label: '평균 dx ≈ GT', pass: false });
      items.push({ id: 'epipolar', label: '평균 |dy| ≤ 2 px', pass: false });
    }
    return items;
  }, [wasm.data, wasm.error, prevFrame.data, currFrame.data, cameras.data, result, currIndex]);

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
      input={
        <InputView
          prev={prevFrame.data ?? null}
          curr={currFrame.data ?? null}
          loading={prevFrame.isLoading || currFrame.isLoading}
        />
      }
      output={
        <OutputView
          prev={prevFrame.data ?? null}
          curr={currFrame.data ?? null}
          seeds={seeds?.pts ?? null}
          tracked={result?.tracked ?? null}
          summary={result?.summary ?? null}
          iterSweep={iterSweep}
        />
      }
      verifyItems={verifyItems}
    />
  );
}

function buildVelocityInit(seeds: Float64Array, dx: number, dy: number): Float64Array {
  const out = new Float64Array(seeds.length);
  for (let i = 0; i < seeds.length; i += 3) {
    out[i + 0] = seeds[i + 0] + dx;
    out[i + 1] = seeds[i + 1] + dy;
    out[i + 2] = 1;
  }
  return out;
}

function countTracked(out: Float64Array, total: number): number {
  let n = 0;
  for (let i = 0; i < total; i++) if (out[i * 3 + 2] > 0.5) n++;
  return n;
}

function ParamPanel({
  currIndex,
  setCurrIndex,
  params,
  setParams,
}: {
  currIndex: number;
  setCurrIndex: (n: number) => void;
  params: Step7Params;
  setParams: (next: Step7Params) => void;
}) {
  const update = <K extends keyof Step7Params>(key: K, value: Step7Params[K]) =>
    setParams({ ...params, [key]: value });
  return (
    <section
      style={{
        border: '1px solid #333',
        borderRadius: 6,
        padding: 12,
        background: '#181818',
        display: 'flex',
        flexDirection: 'column',
        gap: 10,
      }}
    >
      <h3 style={sectionTitle}>Detector (prev frame)</h3>
      <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6 }}>
        {DETECTORS.map((d) => (
          <button
            key={d}
            type="button"
            onClick={() => update('detector', d)}
            style={{
              padding: '4px 10px',
              borderRadius: 4,
              border: '1px solid #444',
              background: params.detector === d ? '#2a3d5c' : '#222',
              color: '#eee',
              cursor: 'pointer',
              fontSize: 13,
            }}
          >
            {d}
          </button>
        ))}
      </div>
      <Slider label="curr frame" value={currIndex} min={1} max={FRAME_COUNT - 1} step={1} onChange={setCurrIndex} format={(n) => `#${n - 1} → #${n}`} />
      <Slider label="maxFeatures" value={params.maxFeatures} min={20} max={500} step={10} onChange={(n) => update('maxFeatures', n)} format={(n) => String(n)} />

      <h3 style={sectionTitle}>Init strategy</h3>
      <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6 }}>
        {(['none', 'velocity'] as const).map((s) => (
          <button
            key={s}
            type="button"
            onClick={() => update('initStrategy', s)}
            style={{
              padding: '4px 10px',
              borderRadius: 4,
              border: '1px solid #444',
              background: params.initStrategy === s ? '#2a3d5c' : '#222',
              color: '#eee',
              cursor: 'pointer',
              fontSize: 13,
            }}
          >
            {s}
          </button>
        ))}
        <span
          title="Map-projection init requires an estimated relative pose — available once Step 8 (PnP) lands."
          style={{
            padding: '4px 10px',
            borderRadius: 4,
            border: '1px dashed #444',
            background: '#1a1a1a',
            color: '#666',
            fontSize: 13,
            cursor: 'not-allowed',
          }}
        >
          map-projection (Step 8)
        </span>
      </div>
      <Slider
        label="velocity dx (px)"
        value={params.velocityDx}
        min={-30}
        max={30}
        step={0.5}
        onChange={(n) => update('velocityDx', n)}
        format={(n) => n.toFixed(1)}
        disabled={params.initStrategy !== 'velocity'}
      />
      <Slider
        label="velocity dy (px)"
        value={params.velocityDy}
        min={-15}
        max={15}
        step={0.5}
        onChange={(n) => update('velocityDy', n)}
        format={(n) => n.toFixed(1)}
        disabled={params.initStrategy !== 'velocity'}
      />

      <h3 style={sectionTitle}>LK pyramid</h3>
      <Slider label="winSize" value={params.winSize} min={5} max={31} step={2} onChange={(n) => update('winSize', n)} format={(n) => String(n)} />
      <Slider label="maxLevel" value={params.maxLevel} min={0} max={5} step={1} onChange={(n) => update('maxLevel', n)} format={(n) => String(n)} />
      <Slider label="maxIter" value={params.maxIter} min={1} max={60} step={1} onChange={(n) => update('maxIter', n)} format={(n) => String(n)} />
      <Slider label="eps" value={params.eps} min={0.001} max={0.1} step={0.001} onChange={(n) => update('eps', n)} format={(n) => n.toFixed(3)} />

      <button
        type="button"
        onClick={() => setParams(DEFAULT_PARAMS)}
        style={{
          padding: '4px 10px',
          borderRadius: 4,
          border: '1px solid #444',
          background: '#222',
          color: '#eee',
          cursor: 'pointer',
          fontSize: 12,
          alignSelf: 'flex-start',
        }}
      >
        reset to ch13 defaults
      </button>
      <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
        ch13 TrackLastFrame: winSize 11, levels 3, iter 30, eps 0.01, OPTFLOW_USE_INITIAL_FLOW.
        합성 fixture GT shift = −7 px @ 0.5× downsample.
      </div>
    </section>
  );
}

function Slider({
  label,
  value,
  min,
  max,
  step,
  onChange,
  format,
  disabled,
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  onChange: (n: number) => void;
  format: (n: number) => string;
  disabled?: boolean;
}) {
  return (
    <label style={{ ...labelStyle, opacity: disabled ? 0.5 : 1 }}>
      <span style={{ display: 'flex', justifyContent: 'space-between' }}>
        <span>{label}</span>
        <strong>{format(value)}</strong>
      </span>
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        disabled={disabled}
        onChange={(e) => onChange(Number(e.target.value))}
      />
    </label>
  );
}

function InputView({
  prev,
  curr,
  loading,
}: {
  prev: { width: number; height: number; left: ImageData; index: number } | null;
  curr: { width: number; height: number; left: ImageData; index: number } | null;
  loading: boolean;
}) {
  const prevRef = useRef<HTMLCanvasElement | null>(null);
  const currRef = useRef<HTMLCanvasElement | null>(null);
  useEffect(() => {
    if (prev && prevRef.current) {
      prevRef.current.width = prev.width;
      prevRef.current.height = prev.height;
      prevRef.current.getContext('2d')?.putImageData(prev.left, 0, 0);
    }
    if (curr && currRef.current) {
      currRef.current.width = curr.width;
      currRef.current.height = curr.height;
      currRef.current.getContext('2d')?.putImageData(curr.left, 0, 0);
    }
  }, [prev, curr]);
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      <div style={{ fontSize: 12, color: '#888' }}>
        {loading ? 'loading…' : prev && curr ? `prev #${prev.index} → curr #${curr.index} · ${curr.width}×${curr.height}` : '—'}
      </div>
      <figure style={figureStyle}>
        <figcaption style={figCaptionStyle}>image_0 prev</figcaption>
        <canvas ref={prevRef} style={canvasStyle} />
      </figure>
      <figure style={figureStyle}>
        <figcaption style={figCaptionStyle}>image_0 curr</figcaption>
        <canvas ref={currRef} style={canvasStyle} />
      </figure>
    </div>
  );
}

function OutputView({
  prev,
  curr,
  seeds,
  tracked,
  summary,
  iterSweep,
}: {
  prev: { width: number; height: number; left: ImageData } | null;
  curr: { width: number; height: number; left: ImageData } | null;
  seeds: Float64Array | null;
  tracked: Float64Array | null;
  summary: MatchSummary | null;
  iterSweep: IterSweepRow[] | null;
}) {
  const ref = useRef<HTMLCanvasElement | null>(null);
  useEffect(() => {
    if (!prev || !curr || !ref.current) return;
    const canvas = ref.current;
    canvas.width = curr.width;
    canvas.height = prev.height + curr.height;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.putImageData(prev.left, 0, 0);
    ctx.putImageData(curr.left, 0, prev.height);
    if (!seeds || !tracked) return;
    const total = seeds.length / 3;
    ctx.lineWidth = 1;
    for (let i = 0; i < total; i++) {
      const sx = seeds[i * 3 + 0];
      const sy = seeds[i * 3 + 1];
      const tx = tracked[i * 3 + 0];
      const ty = tracked[i * 3 + 1] + prev.height;
      const ok = tracked[i * 3 + 2] > 0.5;
      ctx.strokeStyle = ok ? 'rgba(60, 220, 120, 0.7)' : 'rgba(255, 90, 90, 0.5)';
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(tx, ty);
      ctx.stroke();
      ctx.fillStyle = ok ? '#33ff66' : '#ff5050';
      ctx.beginPath();
      ctx.arc(sx, sy, 2.5, 0, Math.PI * 2);
      ctx.fill();
      if (ok) {
        ctx.beginPath();
        ctx.arc(tx, ty, 2.5, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  }, [prev, curr, seeds, tracked]);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      {summary && (
        <div style={{ fontSize: 12, color: '#aaa' }}>
          tracked <strong>{summary.trackedCount}</strong> / {summary.totalCount} ·{' '}
          mean dx = <strong>{summary.meanDx.toFixed(2)}</strong> px ·{' '}
          mean |dy| = {summary.meanAbsDy.toFixed(2)} px ·{' '}
          detect {summary.detectMs.toFixed(1)} ms · LK {summary.ms.toFixed(1)} ms
        </div>
      )}
      <canvas ref={ref} style={canvasStyle} />
      <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
        ● 좌상단 = prev, 좌하단 = curr (수직 stack). 녹색 = 추적 성공, 빨강 = 실패.
        합성 fixture는 프레임당 −7 px (downsample 0.5) 좌측 이동 → 평균 dx ≈ −7 px이 GT.
      </div>
      {iterSweep && iterSweep.length > 0 && (
        <ConvergenceChart rows={iterSweep} totalCount={summary?.totalCount ?? 0} />
      )}
    </div>
  );
}

function ConvergenceChart({ rows, totalCount }: { rows: IterSweepRow[]; totalCount: number }) {
  // Compact SVG bar+line chart so we don't pull a charting dep just for this.
  const w = 280;
  const h = 120;
  const pad = { l: 28, r: 8, t: 8, b: 22 };
  const innerW = w - pad.l - pad.r;
  const innerH = h - pad.t - pad.b;
  const maxY = Math.max(totalCount, 1);
  const xFor = (i: number) => pad.l + (innerW * i) / Math.max(rows.length - 1, 1);
  const yFor = (v: number) => pad.t + innerH - (innerH * v) / maxY;
  const noneLine = rows.map((r, i) => `${i === 0 ? 'M' : 'L'} ${xFor(i)} ${yFor(r.noneTracked)}`).join(' ');
  const velLine = rows.map((r, i) => `${i === 0 ? 'M' : 'L'} ${xFor(i)} ${yFor(r.velocityTracked)}`).join(' ');
  return (
    <figure style={{ margin: 0 }}>
      <figcaption style={{ ...figCaptionStyle, marginBottom: 4 }}>
        수렴 라운드 비교 — tracked count vs maxIter
      </figcaption>
      <svg width={w} height={h} style={{ background: '#0c0c0c', border: '1px solid #333', borderRadius: 4 }}>
        <line x1={pad.l} y1={pad.t} x2={pad.l} y2={pad.t + innerH} stroke="#444" strokeWidth={1} />
        <line x1={pad.l} y1={pad.t + innerH} x2={pad.l + innerW} y2={pad.t + innerH} stroke="#444" strokeWidth={1} />
        <text x={pad.l - 4} y={pad.t + 4} fill="#888" fontSize={9} textAnchor="end">
          {maxY}
        </text>
        <text x={pad.l - 4} y={pad.t + innerH} fill="#888" fontSize={9} textAnchor="end">
          0
        </text>
        {rows.map((r, i) => (
          <text key={`x-${i}`} x={xFor(i)} y={pad.t + innerH + 12} fill="#888" fontSize={9} textAnchor="middle">
            {r.maxIter}
          </text>
        ))}
        <path d={noneLine} stroke="#ff5050" strokeWidth={1.5} fill="none" />
        <path d={velLine} stroke="#33ff66" strokeWidth={1.5} fill="none" />
        {rows.map((r, i) => (
          <g key={`pts-${i}`}>
            <circle cx={xFor(i)} cy={yFor(r.noneTracked)} r={2.5} fill="#ff5050" />
            <circle cx={xFor(i)} cy={yFor(r.velocityTracked)} r={2.5} fill="#33ff66" />
          </g>
        ))}
        <g transform={`translate(${pad.l + 6}, ${pad.t + 4})`}>
          <rect width={10} height={2} y={4} fill="#ff5050" />
          <text x={14} y={8} fill="#ccc" fontSize={10}>none</text>
          <rect width={10} height={2} y={16} fill="#33ff66" />
          <text x={14} y={20} fill="#ccc" fontSize={10}>velocity</text>
        </g>
      </svg>
      <div style={{ fontSize: 11, color: '#888', marginTop: 4, lineHeight: 1.5 }}>
        x축 = maxIter, y축 = tracked count. velocity init이 적은 반복으로도 빠르게 수렴하면
        차트 곡선이 좌측에서 더 빨리 plateau에 도달한다.
      </div>
    </figure>
  );
}

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

const figureStyle: React.CSSProperties = { margin: 0, display: 'flex', flexDirection: 'column', gap: 2 };
const figCaptionStyle: React.CSSProperties = { fontSize: 11, color: '#888' };
const canvasStyle: React.CSSProperties = {
  width: '100%',
  height: 'auto',
  border: '1px solid #333',
  background: '#000',
  imageRendering: 'pixelated',
};
