import { useEffect, useMemo, useRef, useState } from 'react';
import { useQuery } from '@tanstack/react-query';
import { STEPS } from '../index';
import { StepLayout } from '../../components/StepLayout';
import type { VerifyItem } from '../../components/VerifyGate';
import { loadKittiFrame, parseKittiCalib, type KittiCamera } from '../../lib/kitti';
import { imageDataToGray, loadFeaturesWasm, type FeaturesModule } from '../../wasm/features';
import {
  loadTriangulationWasm,
  makeK,
  makeRectifiedT,
  type TriangulationAlgo,
  type TriangulationModule,
} from '../../wasm/triangulation';

const DATASET_DIR = '/datasets/kitti05-mini';
const FRAME_COUNT = 5;
const ALGOS: TriangulationAlgo[] = ['LinearSVD', 'Midpoint'];

interface Step5Params {
  algo: TriangulationAlgo;
  qualityThreshold: number;
  invertedReturn: boolean;
  maxFeatures: number;
}

const DEFAULT_PARAMS: Step5Params = {
  algo: 'LinearSVD',
  qualityThreshold: 0.01,
  invertedReturn: false,
  maxFeatures: 150,
};

interface TriResult {
  /** stride 5 [x, y, z, qualityMetric, ok] */
  points: Float64Array;
  acceptedCount: number;
  totalCount: number;
  trackedCount: number;
  meanDepth: number;
  medianMetric: number;
  maxMetric: number;
  detectMs: number;
  lkMs: number;
  triMs: number;
}

export function Step05Triangulation() {
  const step = STEPS.find((s) => s.id === 5)!;
  const [frameIndex, setFrameIndex] = useState(0);
  const [params, setParams] = useState<Step5Params>(DEFAULT_PARAMS);

  const downsample = 0.5;
  const cameras = useQuery({
    queryKey: ['kitti', 'calib', DATASET_DIR, downsample],
    queryFn: async () => {
      const res = await fetch(`${DATASET_DIR}/calib.txt`);
      if (!res.ok) throw new Error(`calib.txt: HTTP ${res.status}`);
      return parseKittiCalib(await res.text(), downsample);
    },
  });
  const frame = useQuery({
    queryKey: ['kitti', 'frame', DATASET_DIR, frameIndex, downsample],
    queryFn: () => loadKittiFrame(DATASET_DIR, frameIndex, downsample),
  });
  const features = useQuery<FeaturesModule>({
    queryKey: ['wasm', 'features', 'baseline'],
    queryFn: () => loadFeaturesWasm('baseline'),
  });
  const tri = useQuery<TriangulationModule>({
    queryKey: ['wasm', 'triangulation', 'baseline'],
    queryFn: () => loadTriangulationWasm('baseline'),
  });

  const grayLeft = useMemo(() => (frame.data ? imageDataToGray(frame.data.left) : null), [frame.data]);
  const grayRight = useMemo(() => (frame.data ? imageDataToGray(frame.data.right) : null), [frame.data]);

  const left = cameras.data?.find((c) => c.id === 0) ?? null;
  const right = cameras.data?.find((c) => c.id === 1) ?? null;

  const result = useMemo<TriResult | null>(() => {
    if (!features.data || !tri.data || !frame.data || !grayLeft || !grayRight || !left || !right) {
      return null;
    }
    const t0 = performance.now();
    const seeds = features.data.detect(grayLeft, frame.data.width, frame.data.height, 'GFTT', {
      maxFeatures: params.maxFeatures,
      qualityLevel: 0.01,
      minDistance: 20,
    });
    const detectMs = performance.now() - t0;
    if (seeds.length === 0) {
      return {
        points: new Float64Array(0),
        acceptedCount: 0,
        totalCount: 0,
        trackedCount: 0,
        meanDepth: NaN,
        medianMetric: NaN,
        maxMetric: NaN,
        detectMs,
        lkMs: 0,
        triMs: 0,
      };
    }
    const t1 = performance.now();
    const tracked = features.data.track(grayLeft, grayRight, frame.data.width, frame.data.height, seeds, {
      winSize: 11,
      maxLevel: 3,
      maxIter: 30,
      eps: 0.01,
      useInitialFlow: false,
    });
    const lkMs = performance.now() - t1;

    // KITTI rectified pair: world frame = left camera. Right camera is shifted
    // along x by -baseline (so that right camera centre = (+baseline, 0, 0)
    // expressed in world). The Step 1 parser already gives us that translation.
    const kL = makeK(left.fx, left.fy, left.cx, left.cy);
    const kR = makeK(right.fx, right.fy, right.cx, right.cy);
    const tL = makeRectifiedT([0, 0, 0]);
    const tR = makeRectifiedT(right.t);

    const t2 = performance.now();
    const points = tri.data.triangulate(seeds, tracked, kL, tL, kR, tR, {
      algo: params.algo,
      qualityThreshold: params.qualityThreshold,
      invertedReturn: params.invertedReturn,
    });
    const triMs = performance.now() - t2;

    let trackedCount = 0;
    let acceptedCount = 0;
    let depthSum = 0;
    const metrics: number[] = [];
    let maxMetric = 0;
    const total = seeds.length / 3;
    for (let i = 0; i < total; i++) {
      if (tracked[i * 3 + 2] > 0.5) trackedCount++;
      const ok = points[i * 5 + 4] > 0.5;
      if (ok) {
        acceptedCount++;
        depthSum += points[i * 5 + 2];
      }
      const m = points[i * 5 + 3];
      if (Number.isFinite(m)) {
        metrics.push(m);
        if (m > maxMetric) maxMetric = m;
      }
    }
    metrics.sort((a, b) => a - b);
    const median = metrics.length ? metrics[Math.floor(metrics.length / 2)] : NaN;

    return {
      points,
      acceptedCount,
      totalCount: total,
      trackedCount,
      meanDepth: acceptedCount ? depthSum / acceptedCount : NaN,
      medianMetric: median,
      maxMetric,
      detectMs,
      lkMs,
      triMs,
    };
  }, [features.data, tri.data, frame.data, grayLeft, grayRight, left, right, params]);

  const verifyItems: VerifyItem[] = useMemo(() => {
    const items: VerifyItem[] = [];
    items.push({
      id: 'wasm-loaded',
      label: 'features + triangulation WASM 모듈 로드',
      pass: !!features.data && !!tri.data,
    });
    items.push({
      id: 'stereo-frame',
      label: '좌/우 프레임 + calib(P0/P1) 로드',
      pass: !!frame.data && !!left && !!right,
    });
    if (result) {
      items.push({
        id: 'accepted-count',
        label: '삼각화 성공 점 수 ≥ 30',
        pass: result.acceptedCount >= 30,
        detail: `accepted ${result.acceptedCount} / ${result.totalCount}`,
      });
      items.push({
        id: 'depth-positive',
        label: '평균 depth > 0 (정면)',
        pass: Number.isFinite(result.meanDepth) && result.meanDepth > 0,
        detail: Number.isFinite(result.meanDepth) ? `mean z = ${result.meanDepth.toFixed(2)} m` : undefined,
      });
      items.push({
        id: 'depth-range',
        label: '평균 depth ≤ 100 m (KITTI 시퀀스 합리적)',
        pass: Number.isFinite(result.meanDepth) && result.meanDepth <= 100,
      });
    } else {
      items.push({ id: 'accepted-count', label: '삼각화 성공 점 수 ≥ 30', pass: false, detail: '계산 대기' });
      items.push({ id: 'depth-positive', label: '평균 depth > 0', pass: false });
      items.push({ id: 'depth-range', label: '평균 depth ≤ 100 m', pass: false });
    }
    return items;
  }, [features.data, tri.data, frame.data, left, right, result]);

  return (
    <StepLayout
      step={step}
      paramPanel={
        <ParamPanel
          frameIndex={frameIndex}
          setFrameIndex={setFrameIndex}
          params={params}
          setParams={setParams}
        />
      }
      input={<InputView frame={frame.data ?? null} cameraR={right} />}
      output={<OutputView result={result} cameraL={left} />}
      verifyItems={verifyItems}
    />
  );
}

function ParamPanel({
  frameIndex,
  setFrameIndex,
  params,
  setParams,
}: {
  frameIndex: number;
  setFrameIndex: (n: number) => void;
  params: Step5Params;
  setParams: (next: Step5Params) => void;
}) {
  const update = <K extends keyof Step5Params>(key: K, value: Step5Params[K]) =>
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
      <h3 style={sectionTitle}>Algorithm</h3>
      <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6 }}>
        {ALGOS.map((a) => (
          <button
            key={a}
            type="button"
            onClick={() => update('algo', a)}
            style={{
              padding: '4px 10px',
              borderRadius: 4,
              border: '1px solid #444',
              background: params.algo === a ? '#2a3d5c' : '#222',
              color: '#eee',
              cursor: 'pointer',
              fontSize: 13,
            }}
          >
            {a}
          </button>
        ))}
      </div>

      <Slider label="frame" value={frameIndex} min={0} max={FRAME_COUNT - 1} step={1} onChange={setFrameIndex} format={(n) => String(n)} />
      <Slider label="maxFeatures (GFTT seed)" value={params.maxFeatures} min={20} max={500} step={10} onChange={(n) => update('maxFeatures', n)} format={(n) => String(n)} />
      <Slider
        label={params.algo === 'LinearSVD' ? 'σ4/σ3 threshold' : 'rayDist/depth threshold'}
        value={params.qualityThreshold}
        min={1e-4}
        max={0.5}
        step={1e-4}
        onChange={(n) => update('qualityThreshold', n)}
        format={(n) => n.toExponential(1)}
      />

      <label style={{ display: 'flex', gap: 6, alignItems: 'center', fontSize: 13, color: '#ccc' }}>
        <input
          type="checkbox"
          checked={params.invertedReturn}
          onChange={(e) => update('invertedReturn', e.target.checked)}
        />
        invertedReturn (book "포기" 토글)
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
        ch13 algorithm.h: σ4/σ3 &lt; 1e-2 면 "well-conditioned" (수용). invertedReturn 토글은 원서
        주석("해의 품질이 좋지 않아 포기합니다")과 코드의 polarity 충돌을 직접 관찰할 수 있게 한다.
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
  cameraR,
}: {
  frame: { width: number; height: number; left: ImageData; right: ImageData; index: number } | null;
  cameraR: KittiCamera | null;
}) {
  const ref = useRef<HTMLCanvasElement | null>(null);
  useEffect(() => {
    if (!frame || !ref.current) return;
    const canvas = ref.current;
    canvas.width = frame.width;
    canvas.height = frame.height * 2;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.putImageData(frame.left, 0, 0);
    ctx.putImageData(frame.right, 0, frame.height);
  }, [frame]);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      <div style={{ fontSize: 12, color: '#888' }}>
        {frame ? `frame #${frame.index} · ${frame.width}×${frame.height}` : '—'}
        {cameraR && <> · baseline = {cameraR.baseline.toFixed(4)} m</>}
      </div>
      <canvas ref={ref} style={canvasStyle} />
      <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
        상단 = left, 하단 = right. Step 3·4 파이프라인을 그대로 재사용해 GFTT seed → LK → 삼각화로 흐른다.
      </div>
    </div>
  );
}

function OutputView({
  result,
  cameraL,
}: {
  result: TriResult | null;
  cameraL: KittiCamera | null;
}) {
  const topRef = useRef<HTMLCanvasElement | null>(null);
  const histRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    if (!topRef.current) return;
    const canvas = topRef.current;
    const W = 360;
    const H = 240;
    canvas.width = W;
    canvas.height = H;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.fillStyle = '#0d1117';
    ctx.fillRect(0, 0, W, H);
    // Grid (1m steps in z, 1m steps in x). Extents: x in [-15,15], z in [0,40].
    const minX = -15;
    const maxX = 15;
    const minZ = 0;
    const maxZ = 40;
    const xToCanvas = (x: number) => ((x - minX) / (maxX - minX)) * W;
    const zToCanvas = (z: number) => H - ((z - minZ) / (maxZ - minZ)) * H;
    ctx.strokeStyle = '#262a35';
    ctx.lineWidth = 1;
    for (let x = minX; x <= maxX; x += 5) {
      const px = xToCanvas(x);
      ctx.beginPath();
      ctx.moveTo(px, 0);
      ctx.lineTo(px, H);
      ctx.stroke();
    }
    for (let z = minZ; z <= maxZ; z += 5) {
      const py = zToCanvas(z);
      ctx.beginPath();
      ctx.moveTo(0, py);
      ctx.lineTo(W, py);
      ctx.stroke();
    }
    // Camera at origin.
    ctx.fillStyle = '#ddd';
    ctx.beginPath();
    ctx.arc(xToCanvas(0), zToCanvas(0), 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#888';
    ctx.font = '10px monospace';
    ctx.fillText('cam (0,0)', xToCanvas(0) + 6, zToCanvas(0) + 4);
    ctx.fillText('+x →', W - 36, H - 6);
    ctx.fillText('↑ +z', 4, 12);

    if (!result || result.points.length === 0) return;
    const { points, totalCount } = result;
    for (let i = 0; i < totalCount; i++) {
      const x = points[i * 5 + 0];
      const z = points[i * 5 + 2];
      const ok = points[i * 5 + 4] > 0.5;
      if (!Number.isFinite(x) || !Number.isFinite(z)) continue;
      if (z <= 0) continue;
      const cx = xToCanvas(x);
      const cy = zToCanvas(z);
      if (cx < 0 || cx > W || cy < 0 || cy > H) continue;
      ctx.fillStyle = ok ? 'rgba(80, 220, 130, 0.85)' : 'rgba(220, 80, 80, 0.4)';
      ctx.beginPath();
      ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
      ctx.fill();
    }
  }, [result]);

  useEffect(() => {
    if (!histRef.current || !result) return;
    const canvas = histRef.current;
    const W = 360;
    const H = 110;
    canvas.width = W;
    canvas.height = H;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.fillStyle = '#0d1117';
    ctx.fillRect(0, 0, W, H);

    const depths: number[] = [];
    const { points, totalCount } = result;
    for (let i = 0; i < totalCount; i++) {
      if (points[i * 5 + 4] > 0.5) {
        const z = points[i * 5 + 2];
        if (Number.isFinite(z) && z > 0 && z < 80) depths.push(z);
      }
    }
    if (depths.length === 0) {
      ctx.fillStyle = '#777';
      ctx.font = '12px monospace';
      ctx.fillText('depth distribution unavailable', 10, 60);
      return;
    }
    const bins = 32;
    const counts = new Array<number>(bins).fill(0);
    const maxDepth = 60;
    for (const d of depths) {
      const idx = Math.min(bins - 1, Math.floor((d / maxDepth) * bins));
      counts[idx]++;
    }
    const peak = Math.max(...counts);
    const barW = W / bins;
    for (let b = 0; b < bins; b++) {
      const h = (counts[b] / peak) * (H - 24);
      ctx.fillStyle = '#3fa68b';
      ctx.fillRect(b * barW, H - 16 - h, barW - 1, h);
    }
    ctx.fillStyle = '#888';
    ctx.font = '10px monospace';
    ctx.fillText('0', 2, H - 4);
    ctx.fillText(`${maxDepth}m`, W - 28, H - 4);
    ctx.fillText('depth (z, m)', W / 2 - 30, H - 4);
  }, [result]);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      {result && (
        <div style={{ fontSize: 12, color: '#aaa' }}>
          tracked <strong>{result.trackedCount}</strong> / {result.totalCount} ·{' '}
          accepted <strong>{result.acceptedCount}</strong> ·{' '}
          mean z = {Number.isFinite(result.meanDepth) ? `${result.meanDepth.toFixed(2)} m` : '—'} ·{' '}
          median metric = {Number.isFinite(result.medianMetric) ? result.medianMetric.toExponential(2) : '—'}
        </div>
      )}
      {result && (
        <div style={{ fontSize: 11, color: 'var(--color-fg-faint)' }}>
          detect {result.detectMs.toFixed(1)} ms · LK {result.lkMs.toFixed(1)} ms · triangulate {result.triMs.toFixed(1)} ms
          {cameraL && <> · K_l fx = {cameraL.fx.toFixed(2)}</>}
        </div>
      )}
      <figure style={{ margin: 0 }}>
        <figcaption style={figCaptionStyle}>top-down (X, Z) — green = accepted, red = rejected</figcaption>
        <canvas ref={topRef} style={canvasStyle} />
      </figure>
      <figure style={{ margin: 0 }}>
        <figcaption style={figCaptionStyle}>depth histogram (accepted only)</figcaption>
        <canvas ref={histRef} style={canvasStyle} />
      </figure>
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

const figCaptionStyle: React.CSSProperties = { fontSize: 11, color: '#888' };

const canvasStyle: React.CSSProperties = {
  width: '100%',
  height: 'auto',
  border: '1px solid #333',
  background: '#000',
  imageRendering: 'pixelated',
};
