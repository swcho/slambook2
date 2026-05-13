import { useEffect, useMemo, useRef, useState } from 'react';
import { useQuery } from '@tanstack/react-query';
import { STEPS } from '../index';
import { StepLayout } from '../../components/StepLayout';
import type { VerifyItem } from '../../components/VerifyGate';
import { loadKittiCalibFor, loadKittiFrame } from '../../lib/kitti';
import { useDataset } from '../../lib/useDataset';
import {
  imageDataToGray,
  loadFeaturesWasm,
  type DetectorKey,
  type FeaturesModule,
} from '../../wasm/features';

const DETECTORS: DetectorKey[] = ['GFTT', 'Harris', 'FAST', 'ORB'];

interface Step4Params {
  detector: DetectorKey;
  maxFeatures: number;
  qualityLevel: number;
  minDistance: number;
  winSize: number;
  maxLevel: number;
  maxIter: number;
  eps: number;
  useInitialFlow: boolean;
}

const DEFAULT_PARAMS: Step4Params = {
  detector: 'GFTT',
  maxFeatures: 150,
  qualityLevel: 0.01,
  minDistance: 20,
  winSize: 11, // ch13's FindFeaturesInRight uses 11
  maxLevel: 3,
  maxIter: 30,
  eps: 0.01,
  useInitialFlow: false,
};

interface MatchSummary {
  trackedCount: number;
  totalCount: number;
  meanDx: number; // signed mean horizontal disparity (px)
  meanDy: number; // signed mean vertical residual (epipolar deviation, px)
  meanAbsDy: number;
  ms: number;
  detectMs: number;
}

export function Step04StereoMatching() {
  const step = STEPS.find((s) => s.id === 4)!;
  const { dataset } = useDataset();
  const frameCount = dataset.frameCount;
  const [frameIndex, setFrameIndex] = useState(0);
  const clampedFrame = Math.min(frameIndex, frameCount - 1);
  const [params, setParams] = useState<Step4Params>(DEFAULT_PARAMS);

  const downsample = 0.5;
  const cameras = useQuery({
    queryKey: ['kitti', 'calib', dataset.id, downsample],
    queryFn: () => loadKittiCalibFor(dataset, downsample),
  });
  const frame = useQuery({
    queryKey: ['kitti', 'frame', dataset.id, clampedFrame, downsample],
    queryFn: () => loadKittiFrame(dataset.dir, clampedFrame, downsample),
  });
  const wasm = useQuery<FeaturesModule>({
    queryKey: ['wasm', 'features', 'baseline'],
    queryFn: () => loadFeaturesWasm('baseline'),
  });

  const grayLeft = useMemo(() => (frame.data ? imageDataToGray(frame.data.left) : null), [frame.data]);
  const grayRight = useMemo(() => (frame.data ? imageDataToGray(frame.data.right) : null), [frame.data]);

  // Stereo baseline-derived expected disparity for the visual hint:
  // expected dx ≈ -fx * baseline / depth. We don't know depth, so just show
  // baseline magnitude in metres for context.
  const stereoBaseline = cameras.data?.find((c) => c.id === 1)?.baseline ?? null;

  // Detect on left, then LK to right. Re-runs whenever any input changes.
  const result = useMemo(() => {
    if (!wasm.data || !frame.data || !grayLeft || !grayRight) return null;
    const t0 = performance.now();
    const seeds = wasm.data.detect(
      grayLeft,
      frame.data.width,
      frame.data.height,
      params.detector,
      {
        maxFeatures: params.maxFeatures,
        qualityLevel: params.qualityLevel,
        minDistance: params.minDistance,
      },
    );
    const detectMs = performance.now() - t0;
    if (seeds.length === 0) {
      const empty: MatchSummary = {
        trackedCount: 0,
        totalCount: 0,
        meanDx: NaN,
        meanDy: NaN,
        meanAbsDy: NaN,
        ms: 0,
        detectMs,
      };
      return { seeds, tracked: new Float64Array(0), summary: empty };
    }
    // Build initial guesses for the right image: shift seeds left by an
    // educated guess (KITTI baseline × focal / mid-depth ≈ tens of px).
    // Without depth we approximate with stereo baseline in pixels: use 0
    // when useInitialFlow=false (LK starts at the seed location, ch13's
    // default `kps_right.push_back(kp->position_.pt)` branch).
    const initialPts = new Float64Array(seeds.length);
    for (let i = 0; i < seeds.length; i += 3) {
      initialPts[i + 0] = seeds[i + 0];
      initialPts[i + 1] = seeds[i + 1];
      initialPts[i + 2] = 1;
    }
    const t1 = performance.now();
    const tracked = wasm.data.track(grayLeft, grayRight, frame.data.width, frame.data.height, seeds, {
      winSize: params.winSize,
      maxLevel: params.maxLevel,
      maxIter: params.maxIter,
      eps: params.eps,
      useInitialFlow: params.useInitialFlow,
      initialPts: params.useInitialFlow ? initialPts : undefined,
    });
    const ms = performance.now() - t1;
    let trackedCount = 0;
    let dxSum = 0;
    let dySum = 0;
    let absDySum = 0;
    const total = seeds.length / 3;
    for (let i = 0; i < total; i++) {
      const status = tracked[i * 3 + 2];
      if (status > 0.5) {
        trackedCount++;
        const dx = tracked[i * 3 + 0] - seeds[i * 3 + 0];
        const dy = tracked[i * 3 + 1] - seeds[i * 3 + 1];
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
      detectMs,
    };
    return { seeds, tracked, summary };
  }, [wasm.data, frame.data, grayLeft, grayRight, params]);

  const verifyItems: VerifyItem[] = useMemo(() => {
    const items: VerifyItem[] = [];
    items.push({
      id: 'wasm-loaded',
      label: 'features WASM 모듈 로드',
      pass: !!wasm.data,
      detail: wasm.data ? `OpenCV ${wasm.data.opencvVersion}` : wasm.error ? String(wasm.error) : '로딩 중…',
    });
    items.push({
      id: 'stereo-frame',
      label: '좌/우 프레임 + calib 로드',
      pass: !!frame.data && !!cameras.data,
    });
    if (result?.summary) {
      const s = result.summary;
      const matchRate = s.totalCount ? s.trackedCount / s.totalCount : 0;
      items.push({
        id: 'match-rate',
        label: '매칭율 ≥ 60%',
        pass: matchRate >= 0.6,
        detail: `${s.trackedCount} / ${s.totalCount} = ${(matchRate * 100).toFixed(1)}%`,
      });
      items.push({
        id: 'epipolar',
        label: '평균 epipolar (수직) 오차 ≤ 2 px',
        pass: Number.isFinite(s.meanAbsDy) && s.meanAbsDy <= 2,
        detail: Number.isFinite(s.meanAbsDy) ? `mean |dy| = ${s.meanAbsDy.toFixed(2)} px` : undefined,
      });
      items.push({
        id: 'disparity-sign',
        label: '평균 disparity dx < 0 (right shifted left)',
        pass: Number.isFinite(s.meanDx) && s.meanDx < 0,
        detail: Number.isFinite(s.meanDx) ? `mean dx = ${s.meanDx.toFixed(2)} px` : undefined,
      });
    } else {
      items.push({ id: 'match-rate', label: '매칭율 ≥ 60%', pass: false, detail: '계산 대기' });
      items.push({ id: 'epipolar', label: '평균 epipolar 오차 ≤ 2 px', pass: false });
      items.push({ id: 'disparity-sign', label: '평균 disparity dx < 0', pass: false });
    }
    return items;
  }, [wasm.data, wasm.error, frame.data, cameras.data, result]);

  return (
    <StepLayout
      step={step}
      paramPanel={
        <ParamPanel
          frameIndex={clampedFrame}
          setFrameIndex={setFrameIndex}
          frameCount={frameCount}
          params={params}
          setParams={setParams}
        />
      }
      input={
        <InputView
          frame={frame.data ?? null}
          stereoBaseline={stereoBaseline}
          loading={frame.isLoading || frame.isFetching}
        />
      }
      output={
        <OutputView
          frame={frame.data ?? null}
          result={result}
        />
      }
      verifyItems={verifyItems}
    />
  );
}

function ParamPanel({
  frameIndex,
  setFrameIndex,
  frameCount,
  params,
  setParams,
}: {
  frameIndex: number;
  setFrameIndex: (n: number) => void;
  frameCount: number;
  params: Step4Params;
  setParams: (next: Step4Params) => void;
}) {
  const update = <K extends keyof Step4Params>(key: K, value: Step4Params[K]) =>
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
      <h3 style={sectionTitle}>Detector</h3>
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
      <Slider label="frame" value={frameIndex} min={0} max={Math.max(0, frameCount - 1)} step={1} onChange={setFrameIndex} format={(n) => String(n)} />
      <Slider label="maxFeatures" value={params.maxFeatures} min={20} max={500} step={10} onChange={(n) => update('maxFeatures', n)} format={(n) => String(n)} />

      <h3 style={sectionTitle}>LK pyramid</h3>
      <Slider label="winSize" value={params.winSize} min={5} max={31} step={2} onChange={(n) => update('winSize', n)} format={(n) => String(n)} />
      <Slider label="maxLevel" value={params.maxLevel} min={0} max={5} step={1} onChange={(n) => update('maxLevel', n)} format={(n) => String(n)} />
      <Slider label="maxIter" value={params.maxIter} min={5} max={60} step={1} onChange={(n) => update('maxIter', n)} format={(n) => String(n)} />
      <Slider label="eps" value={params.eps} min={0.001} max={0.1} step={0.001} onChange={(n) => update('eps', n)} format={(n) => n.toFixed(3)} />
      <label style={{ display: 'flex', gap: 6, alignItems: 'center', fontSize: 13, color: '#ccc' }}>
        <input
          type="checkbox"
          checked={params.useInitialFlow}
          onChange={(e) => update('useInitialFlow', e.target.checked)}
        />
        useInitialFlow (seed = left position)
      </label>

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
        ch13 FindFeaturesInRight: winSize 11, levels 3, iter 30, eps 0.01, OPTFLOW_USE_INITIAL_FLOW.
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
  frame,
  stereoBaseline,
  loading,
}: {
  frame: { width: number; height: number; left: ImageData; right: ImageData; index: number } | null;
  stereoBaseline: number | null;
  loading: boolean;
}) {
  const leftRef = useRef<HTMLCanvasElement | null>(null);
  const rightRef = useRef<HTMLCanvasElement | null>(null);
  useEffect(() => {
    if (!frame) return;
    if (leftRef.current) {
      leftRef.current.width = frame.width;
      leftRef.current.height = frame.height;
      leftRef.current.getContext('2d')?.putImageData(frame.left, 0, 0);
    }
    if (rightRef.current) {
      rightRef.current.width = frame.width;
      rightRef.current.height = frame.height;
      rightRef.current.getContext('2d')?.putImageData(frame.right, 0, 0);
    }
  }, [frame]);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      <div style={{ fontSize: 12, color: '#888' }}>
        {loading ? 'loading…' : frame ? `frame #${frame.index} · ${frame.width}×${frame.height}` : '—'}
        {stereoBaseline != null && <> · baseline = {stereoBaseline.toFixed(4)} m</>}
      </div>
      <figure style={figureStyle}>
        <figcaption style={figCaptionStyle}>image_0 (left)</figcaption>
        <canvas ref={leftRef} style={canvasStyle} />
      </figure>
      <figure style={figureStyle}>
        <figcaption style={figCaptionStyle}>image_1 (right)</figcaption>
        <canvas ref={rightRef} style={canvasStyle} />
      </figure>
    </div>
  );
}

function OutputView({
  frame,
  result,
}: {
  frame: { width: number; height: number; left: ImageData; right: ImageData } | null;
  result: { seeds: Float64Array; tracked: Float64Array; summary: MatchSummary } | null;
}) {
  const ref = useRef<HTMLCanvasElement | null>(null);
  useEffect(() => {
    if (!frame || !ref.current) return;
    const canvas = ref.current;
    // Stack left over right vertically so a single canvas can show
    // correspondence lines that cross between the two images.
    canvas.width = frame.width;
    canvas.height = frame.height * 2;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.putImageData(frame.left, 0, 0);
    ctx.putImageData(frame.right, 0, frame.height);
    if (!result) return;
    const { seeds, tracked } = result;
    const total = seeds.length / 3;
    ctx.lineWidth = 1;
    for (let i = 0; i < total; i++) {
      const sx = seeds[i * 3 + 0];
      const sy = seeds[i * 3 + 1];
      const tx = tracked[i * 3 + 0];
      const ty = tracked[i * 3 + 1] + frame.height;
      const ok = tracked[i * 3 + 2] > 0.5;
      ctx.strokeStyle = ok ? 'rgba(60, 220, 120, 0.7)' : 'rgba(255, 90, 90, 0.5)';
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(tx, ty);
      ctx.stroke();
      ctx.beginPath();
      ctx.fillStyle = ok ? '#33ff66' : '#ff5050';
      ctx.arc(sx, sy, 2.5, 0, Math.PI * 2);
      ctx.fill();
      if (ok) {
        ctx.beginPath();
        ctx.arc(tx, ty, 2.5, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  }, [frame, result]);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      {result?.summary && (
        <div style={{ fontSize: 12, color: '#aaa' }}>
          tracked <strong>{result.summary.trackedCount}</strong> / {result.summary.totalCount} ·{' '}
          mean dx = <strong>{result.summary.meanDx.toFixed(2)}</strong> px ·{' '}
          mean |dy| = {result.summary.meanAbsDy.toFixed(2)} px ·{' '}
          detect {result.summary.detectMs.toFixed(1)} ms · LK {result.summary.ms.toFixed(1)} ms
        </div>
      )}
      <canvas ref={ref} style={canvasStyle} />
      <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
        ● 좌상단 = left, 좌하단 = right (수직 stack). 녹색 선/점 = 추적 성공, 빨강 = 실패.
        평균 disparity가 음수여야 정상(우측이 좌로 이동), 평균 |dy| ≤ 2 px이면 epipolar 정렬 양호.
      </div>
    </div>
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
