import { useMemo, useState } from 'react';
import { useQuery } from '@tanstack/react-query';
import { STEPS } from '../index';
import { StepLayout } from '../../components/StepLayout';
import type { VerifyItem } from '../../components/VerifyGate';
import { loadKittiFrame, parseKittiCalib, type KittiCamera, type StereoFrame } from '../../lib/kitti';
import {
  imageDataToGray,
  loadFeaturesWasm,
  type FeaturesModule,
} from '../../wasm/features';
import {
  loadTriangulationWasm,
  makeK,
  makeRectifiedT,
  type TriangulationModule,
} from '../../wasm/triangulation';
import { Scene3D, type CameraFrustum, type PointCloudInput } from '../../components/Scene3D';

// Step 10 mirrors ch13/src/frontend.cpp::TriangulateNewPoints. The book's
// flow: when a new KF arrives it (1) sets observations for surviving features,
// (2) re-detects on the left image with a mask covering existing features —
// `cv::rectangle(mask, pt-10..pt+10, 0, FILLED)` — (3) does stereo LK on the
// new detections, (4) triangulates new MapPoints. PLAN §3 Step 10 exposes the
// "redetect-only-new vs full-redetect" toggle as the educational comparison.

const DATASET_DIR = '/datasets/kitti05-mini';
const FRAME_COUNT = 5;

type RedetectMode = 'mask-existing' | 'full-redetect';

interface Step10Params {
  newKeyframe: number;
  redetect: RedetectMode;
  /** Mask radius around each existing feature in pixels (ch13 uses ±10 box). */
  maskRadius: number;
  maxFeatures: number;
  qualityLevel: number;
  /** Reject new MapPoints whose depth (z, current camera) falls outside this range. */
  depthMin: number;
  depthMax: number;
}

const DEFAULT_PARAMS: Step10Params = {
  newKeyframe: 1,
  redetect: 'mask-existing',
  maskRadius: 10,
  maxFeatures: 200,
  qualityLevel: 0.01,
  depthMin: 1,
  depthMax: 80,
};

interface KFRunResult {
  carryCount: number;
  newCount: number;
  rejectedCount: number;
  carryPoints2d: Float64Array;
  newPoints2d: Float64Array;
  carryPoints3d: Float32Array;
  newPoints3d: Float32Array;
  detectMs: number;
  stereoMs: number;
  triMs: number;
  trackMs: number;
}

interface SequenceRow {
  newKeyframe: number;
  carryCount: number;
  newCount: number;
}

export function Step10NewMapPoints() {
  const step = STEPS.find((s) => s.id === 10)!;
  const [params, setParams] = useState<Step10Params>(DEFAULT_PARAMS);

  const downsample = 0.5;
  const cameras = useQuery({
    queryKey: ['kitti', 'calib', DATASET_DIR, downsample],
    queryFn: async () => {
      const res = await fetch(`${DATASET_DIR}/calib.txt`);
      if (!res.ok) throw new Error(`calib.txt: HTTP ${res.status}`);
      return parseKittiCalib(await res.text(), downsample);
    },
  });
  const frames = useQuery({
    queryKey: ['kitti', 'all-frames', DATASET_DIR, downsample],
    queryFn: async () => {
      const out: StereoFrame[] = [];
      for (let i = 0; i < FRAME_COUNT; i++) {
        out.push(await loadKittiFrame(DATASET_DIR, i, downsample));
      }
      return out;
    },
  });
  const features = useQuery<FeaturesModule>({
    queryKey: ['wasm', 'features', 'baseline'],
    queryFn: () => loadFeaturesWasm('baseline'),
  });
  const tri = useQuery<TriangulationModule>({
    queryKey: ['wasm', 'triangulation', 'baseline'],
    queryFn: () => loadTriangulationWasm('baseline'),
  });

  const left = cameras.data?.find((c) => c.id === 0) ?? null;
  const right = cameras.data?.find((c) => c.id === 1) ?? null;

  // Live result for the selected newKeyframe.
  const result = useMemo<KFRunResult | null>(() => {
    if (!features.data || !tri.data || !frames.data || !left || !right) return null;
    if (params.newKeyframe < 1 || params.newKeyframe >= frames.data.length) return null;
    return runNewKeyframe(
      features.data,
      tri.data,
      frames.data[0],
      frames.data[params.newKeyframe],
      left,
      right,
      params,
    );
  }, [features.data, tri.data, frames.data, left, right, params]);

  // Bar chart across all KF candidates (frames 1..N-1) for the "new MapPoint
  // per KF" log graph PLAN §3 Step 10 calls out. Recomputed only when params
  // affecting detection change.
  const sequence = useMemo<SequenceRow[] | null>(() => {
    if (!features.data || !tri.data || !frames.data || !left || !right) return null;
    const rows: SequenceRow[] = [];
    for (let i = 1; i < frames.data.length; i++) {
      const r = runNewKeyframe(
        features.data,
        tri.data,
        frames.data[0],
        frames.data[i],
        left,
        right,
        { ...params, newKeyframe: i },
      );
      if (r) rows.push({ newKeyframe: i, carryCount: r.carryCount, newCount: r.newCount });
    }
    return rows;
  }, [
    features.data, tri.data, frames.data, left, right,
    params.redetect, params.maskRadius, params.maxFeatures, params.qualityLevel,
    params.depthMin, params.depthMax,
    // newKeyframe deliberately omitted — sequence shows all KFs at once.
  ]);

  const verifyItems: VerifyItem[] = useMemo(() => {
    const items: VerifyItem[] = [];
    items.push({
      id: 'wasm-loaded',
      label: 'features + triangulation WASM 모듈 로드',
      pass: !!features.data && !!tri.data,
    });
    items.push({
      id: 'frames-loaded',
      label: 'frame 0 + 새 KF 프레임 + calib 로드',
      pass: !!frames.data && !!cameras.data,
    });
    if (result) {
      items.push({
        id: 'new-mp-positive',
        label: '새 MapPoint 수 ≥ 10',
        pass: result.newCount >= 10,
        detail: `${result.newCount} new (${result.rejectedCount} rejected by depth)`,
      });
      items.push({
        id: 'depth-bounded',
        label: `새 MapPoint depth ∈ [${params.depthMin}, ${params.depthMax}] m`,
        pass: result.newCount > 0 && depthsWithin(result.newPoints3d, params.depthMin, params.depthMax),
        detail:
          result.newPoints3d.length > 0
            ? `mean z = ${meanDepth(result.newPoints3d).toFixed(2)} m`
            : undefined,
      });
    } else {
      items.push({ id: 'new-mp-positive', label: '새 MapPoint 수 ≥ 10', pass: false, detail: '계산 대기' });
      items.push({ id: 'depth-bounded', label: 'depth 합리성', pass: false });
    }
    return items;
  }, [features.data, tri.data, frames.data, cameras.data, result, params.depthMin, params.depthMax]);

  return (
    <StepLayout
      step={step}
      paramPanel={<ParamPanel params={params} setParams={setParams} maxFrame={FRAME_COUNT - 1} />}
      input={<InputView frames={frames.data ?? null} kfIndex={params.newKeyframe} />}
      output={
        <OutputView
          frame={frames.data?.[params.newKeyframe] ?? null}
          baseline={right?.baseline ?? 0.5}
          result={result}
          sequence={sequence}
          activeKf={params.newKeyframe}
        />
      }
      verifyItems={verifyItems}
    />
  );
}

function runNewKeyframe(
  features: FeaturesModule,
  tri: TriangulationModule,
  prev: StereoFrame,
  newKf: StereoFrame,
  left: KittiCamera,
  right: KittiCamera,
  params: Step10Params,
): KFRunResult | null {
  const w = newKf.width;
  const h = newKf.height;
  const grayPrevL = imageDataToGray(prev.left);
  const grayKfL = imageDataToGray(newKf.left);
  const grayKfR = imageDataToGray(newKf.right);

  // (1) build the carry-over set: detect on frame 0 left, stereo-triangulate,
  //     then temporal-LK frame 0 left → newKf left to find which survive.
  const seedsPrev = features.detect(grayPrevL, w, h, 'GFTT', {
    maxFeatures: params.maxFeatures,
    qualityLevel: params.qualityLevel,
    minDistance: 20,
  });
  if (seedsPrev.length === 0) return null;

  const grayPrevR = imageDataToGray(prev.right);
  const stereoPrev = features.track(grayPrevL, grayPrevR, w, h, seedsPrev, {
    winSize: 11, maxLevel: 3, maxIter: 30, eps: 0.01,
  });
  const kL = makeK(left.fx, left.fy, left.cx, left.cy);
  const kR = makeK(right.fx, right.fy, right.cx, right.cy);
  const tL = makeRectifiedT([0, 0, 0]);
  const tR = makeRectifiedT(right.t);
  const triPrev = tri.triangulate(seedsPrev, stereoPrev, kL, tL, kR, tR, {
    algo: 'LinearSVD', qualityThreshold: 0.01,
  });

  const t3 = performance.now();
  const trackedPrev = features.track(grayPrevL, grayKfL, w, h, seedsPrev, {
    winSize: 11, maxLevel: 3, maxIter: 30, eps: 0.01,
  });
  const trackMs = performance.now() - t3;

  const total = seedsPrev.length / 3;
  const carry2d: number[] = [];
  const carry3d: number[] = [];
  for (let i = 0; i < total; i++) {
    const triOk = triPrev[i * 5 + 4] > 0.5;
    const lkOk = trackedPrev[i * 3 + 2] > 0.5;
    if (!triOk || !lkOk) continue;
    carry2d.push(trackedPrev[i * 3 + 0], trackedPrev[i * 3 + 1]);
    // Carry-over points were triangulated in frame 0 camera frame. For the
    // synthetic fixture motion is essentially zero (same camera frame as the
    // new KF) so we display them as-is. Real KITTI would apply the relative
    // pose from Step 8 — that wiring lands together with Phase H Map manager.
    carry3d.push(triPrev[i * 5 + 0], triPrev[i * 5 + 1], triPrev[i * 5 + 2]);
  }
  const carryCount = carry2d.length / 2;

  // (2) re-detect on the new KF left, optionally masking the carry-over set.
  const mask =
    params.redetect === 'mask-existing'
      ? buildCircleMask(w, h, carry2d, params.maskRadius)
      : undefined;
  const t0 = performance.now();
  const seedsKf = features.detect(grayKfL, w, h, 'GFTT', {
    maxFeatures: params.maxFeatures,
    qualityLevel: params.qualityLevel,
    minDistance: 20,
    mask,
  });
  const detectMs = performance.now() - t0;

  // (3) stereo LK + triangulate new detections.
  const t1 = performance.now();
  const stereoKf = features.track(grayKfL, grayKfR, w, h, seedsKf, {
    winSize: 11, maxLevel: 3, maxIter: 30, eps: 0.01,
  });
  const stereoMs = performance.now() - t1;
  const t2 = performance.now();
  const triKf = tri.triangulate(seedsKf, stereoKf, kL, tL, kR, tR, {
    algo: 'LinearSVD', qualityThreshold: 0.01,
  });
  const triMs = performance.now() - t2;

  // (4) accept new MapPoints inside the depth band (PLAN §3 Step 10
  //     "depth 합리성 범위" parameter).
  const newTotal = seedsKf.length / 3;
  const new2d: number[] = [];
  const new3d: number[] = [];
  let rejected = 0;
  for (let i = 0; i < newTotal; i++) {
    const ok = triKf[i * 5 + 4] > 0.5;
    if (!ok) {
      rejected++;
      continue;
    }
    const z = triKf[i * 5 + 2];
    if (z < params.depthMin || z > params.depthMax) {
      rejected++;
      continue;
    }
    new2d.push(seedsKf[i * 3 + 0], seedsKf[i * 3 + 1]);
    new3d.push(triKf[i * 5 + 0], triKf[i * 5 + 1], z);
  }

  return {
    carryCount,
    newCount: new2d.length / 2,
    rejectedCount: rejected,
    carryPoints2d: new Float64Array(carry2d),
    newPoints2d: new Float64Array(new2d),
    carryPoints3d: new Float32Array(carry3d),
    newPoints3d: new Float32Array(new3d),
    detectMs,
    stereoMs,
    triMs,
    trackMs,
  };
}

function buildCircleMask(
  width: number,
  height: number,
  pts2d: number[],
  radius: number,
): Uint8Array {
  const out = new Uint8Array(width * height);
  out.fill(255);
  const r2 = radius * radius;
  for (let i = 0; i < pts2d.length; i += 2) {
    const cx = Math.round(pts2d[i + 0]);
    const cy = Math.round(pts2d[i + 1]);
    const x0 = Math.max(0, cx - radius);
    const x1 = Math.min(width - 1, cx + radius);
    const y0 = Math.max(0, cy - radius);
    const y1 = Math.min(height - 1, cy + radius);
    for (let y = y0; y <= y1; y++) {
      const dy = y - cy;
      for (let x = x0; x <= x1; x++) {
        const dx = x - cx;
        if (dx * dx + dy * dy <= r2) out[y * width + x] = 0;
      }
    }
  }
  return out;
}

function meanDepth(pts: Float32Array): number {
  if (pts.length === 0) return 0;
  let s = 0;
  for (let i = 2; i < pts.length; i += 3) s += pts[i];
  return s / (pts.length / 3);
}

function depthsWithin(pts: Float32Array, lo: number, hi: number): boolean {
  for (let i = 2; i < pts.length; i += 3) {
    if (pts[i] < lo || pts[i] > hi) return false;
  }
  return true;
}

function ParamPanel({
  params, setParams, maxFrame,
}: {
  params: Step10Params;
  setParams: (next: Step10Params) => void;
  maxFrame: number;
}) {
  const update = <K extends keyof Step10Params>(key: K, value: Step10Params[K]) =>
    setParams({ ...params, [key]: value });
  return (
    <section style={paramPanelStyle}>
      <h3 style={sectionTitle}>New keyframe</h3>
      <Slider
        label="newKeyframe (vs frame 0)"
        value={params.newKeyframe}
        min={1}
        max={maxFrame}
        step={1}
        onChange={(n) => update('newKeyframe', n)}
        format={(n) => `#0 → #${n}`}
      />

      <h3 style={sectionTitle}>Re-detection mode</h3>
      <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6 }}>
        {(['mask-existing', 'full-redetect'] as const).map((m) => (
          <button
            key={m}
            type="button"
            onClick={() => update('redetect', m)}
            style={{ ...algoBtn, background: params.redetect === m ? '#2a3d5c' : '#222' }}
            title={
              m === 'mask-existing'
                ? 'ch13 default: cv::rectangle(mask, pt±10) for every surviving feature.'
                : 'No mask — every detection competes regardless of carry-over coverage.'
            }
          >
            {m}
          </button>
        ))}
      </div>
      <Slider
        label="mask radius (px)"
        value={params.maskRadius}
        min={2}
        max={30}
        step={1}
        onChange={(n) => update('maskRadius', n)}
        format={(n) => String(n)}
        disabled={params.redetect !== 'mask-existing'}
      />

      <h3 style={sectionTitle}>Detector + depth band</h3>
      <Slider
        label="maxFeatures"
        value={params.maxFeatures}
        min={20}
        max={500}
        step={10}
        onChange={(n) => update('maxFeatures', n)}
        format={(n) => String(n)}
      />
      <Slider
        label="qualityLevel"
        value={params.qualityLevel}
        min={0.001}
        max={0.1}
        step={0.001}
        onChange={(n) => update('qualityLevel', n)}
        format={(n) => n.toFixed(3)}
      />
      <Slider
        label="depthMin (m)"
        value={params.depthMin}
        min={0.1}
        max={20}
        step={0.1}
        onChange={(n) => update('depthMin', n)}
        format={(n) => n.toFixed(1)}
      />
      <Slider
        label="depthMax (m)"
        value={params.depthMax}
        min={5}
        max={200}
        step={1}
        onChange={(n) => update('depthMax', n)}
        format={(n) => n.toFixed(0)}
      />

      <button
        type="button"
        onClick={() => setParams(DEFAULT_PARAMS)}
        style={resetBtn}
      >
        reset (mask radius 10 = ch13 ±10 box)
      </button>
      <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
        ch13 TriangulateNewPoints: 새 KF의 left에서 GFTT 재검출 + cv::rectangle 마스크(±10) +
        stereo LK + linear-SVD triangulation + z &gt; 0 검사. 본 Step은 합성 fixture에서
        carry-over set을 frame 0 stereo 결과로 시뮬레이션한다 (실제 carry-over는 Phase H Map 매니저로 영속).
      </div>
    </section>
  );
}

function Slider({
  label, value, min, max, step, onChange, format, disabled,
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

function InputView({ frames, kfIndex }: { frames: StereoFrame[] | null; kfIndex: number }) {
  if (!frames) return <div style={{ color: 'var(--color-fg-faint)', fontSize: 12 }}>loading…</div>;
  const prev = frames[0];
  const kf = frames[kfIndex];
  if (!prev || !kf) return null;
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
      <div style={{ fontSize: 12, color: '#888' }}>
        prev KF #0 → new KF #{kfIndex} · {kf.width}×{kf.height}
      </div>
      <figure style={figStyle}>
        <figcaption style={figCaption}>frame #0 left (carry-over source)</figcaption>
        <ImageCanvas image={prev.left} />
      </figure>
      <figure style={figStyle}>
        <figcaption style={figCaption}>frame #{kfIndex} left (new KF; mask + redetect)</figcaption>
        <ImageCanvas image={kf.left} />
      </figure>
      <figure style={figStyle}>
        <figcaption style={figCaption}>frame #{kfIndex} right (stereo LK target)</figcaption>
        <ImageCanvas image={kf.right} />
      </figure>
    </div>
  );
}

function ImageCanvas({ image }: { image: ImageData }) {
  return (
    <canvas
      ref={(node) => {
        if (!node) return;
        node.width = image.width;
        node.height = image.height;
        node.getContext('2d')?.putImageData(image, 0, 0);
      }}
      style={canvasStyle}
    />
  );
}

function OutputView({
  frame, baseline, result, sequence, activeKf,
}: {
  frame: StereoFrame | null;
  baseline: number;
  result: KFRunResult | null;
  sequence: SequenceRow[] | null;
  activeKf: number;
}) {
  const frustums: CameraFrustum[] = useMemo(
    () => [
      { worldFromCamera: identityMatrix(), scale: 1.5, color: [0.4, 0.9, 0.5], label: 'KF (left)' },
      { worldFromCamera: shiftedIdentity(baseline), scale: 1.5, color: [0.4, 0.6, 1.0], label: 'KF (right)' },
    ],
    [baseline],
  );
  const cloud: PointCloudInput | null = useMemo(() => {
    if (!result) return null;
    const positions = new Float32Array(result.carryPoints3d.length + result.newPoints3d.length);
    positions.set(result.carryPoints3d, 0);
    positions.set(result.newPoints3d, result.carryPoints3d.length);
    const colors = new Float32Array(positions.length);
    for (let i = 0; i < result.carryPoints3d.length; i += 3) {
      colors[i + 0] = 0.3; colors[i + 1] = 0.85; colors[i + 2] = 0.95; // cyan = carry
    }
    for (let i = result.carryPoints3d.length; i < positions.length; i += 3) {
      colors[i + 0] = 0.95; colors[i + 1] = 0.85; colors[i + 2] = 0.2; // yellow = new
    }
    return { positions, colors, size: 0.2 };
  }, [result]);

  if (!frame || !result || !cloud) {
    return <div style={{ color: 'var(--color-fg-faint)', fontSize: 12 }}>(running re-detection + triangulation…)</div>;
  }
  const totalMs = result.detectMs + result.stereoMs + result.triMs + result.trackMs;

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      <div style={{ fontSize: 12, color: '#aaa' }}>
        carry-over <strong style={{ color: '#5fcfeb' }}>{result.carryCount}</strong> ·{' '}
        new <strong style={{ color: '#f4d04a' }}>{result.newCount}</strong>{' '}
        ({result.rejectedCount} rejected) · detect {result.detectMs.toFixed(1)} ms ·{' '}
        track {result.trackMs.toFixed(1)} ms · stereo {result.stereoMs.toFixed(1)} ms ·{' '}
        tri {result.triMs.toFixed(1)} ms · total <strong>{totalMs.toFixed(1)} ms</strong>
      </div>

      <figure style={figStyle}>
        <figcaption style={figCaption}>
          new KF left — 청록 = carry-over 위치, 노랑 = 새 검출
        </figcaption>
        <KeyframeOverlay frame={frame} result={result} />
      </figure>

      <figure style={figStyle}>
        <figcaption style={figCaption}>3D MapPoints (KF camera frame)</figcaption>
        <Scene3D
          width={400}
          height={260}
          frustums={frustums}
          pointCloud={cloud}
          initialDistance={Math.max(20, meanDepth(result.newPoints3d) * 1.6)}
        />
      </figure>

      {sequence && sequence.length > 0 && (
        <SequenceChart rows={sequence} activeKf={activeKf} />
      )}

      <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
        ● mask-existing 모드는 carry-over 점 주변 ±radius 픽셀에 마스크를 씌워 새 검출이 기존 점과
        겹치지 않도록 한다. full-redetect로 토글하면 새 검출이 carry-over 영역에서도 자유롭게 일어나고
        결과적으로 동일 코너에서 중복 MapPoint가 생긴다 — 학습용으로 비교.
      </div>
    </div>
  );
}

function KeyframeOverlay({ frame, result }: { frame: StereoFrame; result: KFRunResult }) {
  return (
    <canvas
      ref={(node) => {
        if (!node) return;
        node.width = frame.width;
        node.height = frame.height;
        const ctx = node.getContext('2d');
        if (!ctx) return;
        ctx.putImageData(frame.left, 0, 0);
        ctx.fillStyle = 'rgba(95, 207, 235, 0.85)';
        for (let i = 0; i < result.carryPoints2d.length; i += 2) {
          ctx.beginPath();
          ctx.arc(result.carryPoints2d[i], result.carryPoints2d[i + 1], 3, 0, Math.PI * 2);
          ctx.fill();
        }
        ctx.fillStyle = 'rgba(244, 208, 74, 0.9)';
        for (let i = 0; i < result.newPoints2d.length; i += 2) {
          ctx.beginPath();
          ctx.arc(result.newPoints2d[i], result.newPoints2d[i + 1], 3, 0, Math.PI * 2);
          ctx.fill();
        }
      }}
      style={canvasStyle}
    />
  );
}

function SequenceChart({ rows, activeKf }: { rows: SequenceRow[]; activeKf: number }) {
  const w = 360;
  const h = 130;
  const pad = { l: 32, r: 8, t: 14, b: 24 };
  const innerW = w - pad.l - pad.r;
  const innerH = h - pad.t - pad.b;
  const maxY = Math.max(1, ...rows.map((r) => r.carryCount + r.newCount));
  const xFor = (i: number) => pad.l + (innerW * (i + 0.5)) / rows.length;
  const yFor = (v: number) => pad.t + innerH - (innerH * v) / maxY;
  const barWidth = (innerW / rows.length) * 0.55;
  return (
    <figure style={{ margin: 0 }}>
      <figcaption style={{ ...figCaption, marginBottom: 4 }}>
        새 MapPoint per KF (carry vs new, frame 0 → #i)
      </figcaption>
      <svg width={w} height={h} style={{ background: '#0c0c0c', border: '1px solid #333', borderRadius: 4 }}>
        <line x1={pad.l} y1={pad.t} x2={pad.l} y2={pad.t + innerH} stroke="#444" strokeWidth={1} />
        <line x1={pad.l} y1={pad.t + innerH} x2={pad.l + innerW} y2={pad.t + innerH} stroke="#444" strokeWidth={1} />
        <text x={pad.l - 4} y={pad.t + 4} fill="#888" fontSize={9} textAnchor="end">{maxY}</text>
        <text x={pad.l - 4} y={pad.t + innerH} fill="#888" fontSize={9} textAnchor="end">0</text>
        {rows.map((r, i) => {
          const x = xFor(i);
          const total = r.carryCount + r.newCount;
          const yTop = yFor(total);
          const ySplit = yFor(r.newCount);
          const isActive = r.newKeyframe === activeKf;
          return (
            <g key={r.newKeyframe}>
              {/* new on top (yellow), carry stacked below (cyan) */}
              <rect
                x={x - barWidth / 2}
                y={yTop}
                width={barWidth}
                height={ySplit - yTop}
                fill="#f4d04a"
                opacity={isActive ? 1 : 0.7}
              />
              <rect
                x={x - barWidth / 2}
                y={ySplit}
                width={barWidth}
                height={pad.t + innerH - ySplit}
                fill="#5fcfeb"
                opacity={isActive ? 1 : 0.7}
              />
              <text x={x} y={yTop - 3} fill="#ccc" fontSize={9} textAnchor="middle">
                {total}
              </text>
              <text x={x} y={pad.t + innerH + 12} fill={isActive ? '#fff' : '#888'} fontSize={9} textAnchor="middle">
                #0→#{r.newKeyframe}
              </text>
            </g>
          );
        })}
        <g transform={`translate(${pad.l + innerW - 60}, ${pad.t + 4})`}>
          <rect width={9} height={9} y={0} fill="#f4d04a" />
          <text x={12} y={8} fill="#ccc" fontSize={10}>new</text>
          <rect width={9} height={9} y={12} fill="#5fcfeb" />
          <text x={12} y={20} fill="#ccc" fontSize={10}>carry</text>
        </g>
      </svg>
    </figure>
  );
}

function identityMatrix(): number[] {
  return [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ];
}

function shiftedIdentity(baseline: number): number[] {
  // Right camera: world←camera = translate(+baseline, 0, 0). The frustum is
  // drawn in *world* coords, so this just slides it along +X.
  return [
    1, 0, 0, baseline,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ];
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
