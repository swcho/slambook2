import { useMemo, useState } from 'react';
import { useQuery } from '@tanstack/react-query';
import { STEPS } from '../index';
import { StepLayout } from '../../components/StepLayout';
import type { VerifyItem } from '../../components/VerifyGate';
import { Scene3D, type CameraFrustum, type PointCloudInput } from '../../components/Scene3D';
import { loadKittiCalibFor, loadKittiFrame, type KittiCamera } from '../../lib/kitti';
import { useDataset } from '../../lib/useDataset';
import { imageDataToGray, loadFeaturesWasm, type FeaturesModule } from '../../wasm/features';
import {
  loadTriangulationWasm,
  makeK,
  makeRectifiedT,
  type TriangulationModule,
} from '../../wasm/triangulation';

interface Step6Params {
  numFeaturesInit: number; // ch13's num_features_init = 50
  maxFeatures: number;
  qualityThreshold: number;
}

const DEFAULT_PARAMS: Step6Params = {
  numFeaturesInit: 50,
  maxFeatures: 150,
  qualityThreshold: 0.01,
};

interface InitMapResult {
  frameIdx: number;
  /** Float32Array of accepted MapPoint XYZ (length = 3 * landmarkCount). */
  positions: Float32Array;
  landmarkCount: number;
  trackedCount: number;
  detectedCount: number;
  meanDepth: number;
  minDepth: number;
  maxDepth: number;
  /** stride 5 [x, y, z, metric, ok] — full debug output. */
  rawPoints: Float64Array;
}

export function Step06InitialMap() {
  const step = STEPS.find((s) => s.id === 6)!;
  const { dataset } = useDataset();
  const frameCount = dataset.frameCount;
  const [frameIndex, setFrameIndex] = useState(0);
  const clampedFrame = Math.min(frameIndex, frameCount - 1);
  const [params, setParams] = useState<Step6Params>(DEFAULT_PARAMS);

  const downsample = 0.5;
  const cameras = useQuery({
    queryKey: ['kitti', 'calib', dataset.id, downsample],
    queryFn: () => loadKittiCalibFor(dataset, downsample),
  });
  const frame = useQuery({
    queryKey: ['kitti', 'frame', dataset.id, clampedFrame, downsample],
    queryFn: () => loadKittiFrame(dataset.dir, clampedFrame, downsample),
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

  const result = useMemo<InitMapResult | null>(() => {
    if (!features.data || !tri.data || !frame.data || !left || !right) return null;
    const grayLeft = imageDataToGray(frame.data.left);
    const grayRight = imageDataToGray(frame.data.right);
    const seeds = features.data.detect(grayLeft, frame.data.width, frame.data.height, 'GFTT', {
      maxFeatures: params.maxFeatures,
      qualityLevel: 0.01,
      minDistance: 20,
    });
    if (seeds.length === 0) {
      return {
        frameIdx: frame.data.index,
        positions: new Float32Array(0),
        landmarkCount: 0,
        trackedCount: 0,
        detectedCount: 0,
        meanDepth: NaN,
        minDepth: NaN,
        maxDepth: NaN,
        rawPoints: new Float64Array(0),
      };
    }
    const tracked = features.data.track(grayLeft, grayRight, frame.data.width, frame.data.height, seeds, {
      winSize: 11,
      maxLevel: 3,
      maxIter: 30,
      eps: 0.01,
      useInitialFlow: false,
    });
    const kL = makeK(left.fx, left.fy, left.cx, left.cy);
    const kR = makeK(right.fx, right.fy, right.cx, right.cy);
    const tL = makeRectifiedT([0, 0, 0]);
    const tR = makeRectifiedT(right.t);
    const points = tri.data.triangulate(seeds, tracked, kL, tL, kR, tR, {
      algo: 'LinearSVD',
      qualityThreshold: params.qualityThreshold,
      invertedReturn: false,
    });
    const N = seeds.length / 3;
    let trackedCount = 0;
    const accepted: number[] = [];
    for (let i = 0; i < N; i++) {
      if (tracked[i * 3 + 2] > 0.5) trackedCount++;
      if (points[i * 5 + 4] > 0.5) {
        accepted.push(points[i * 5 + 0], points[i * 5 + 1], points[i * 5 + 2]);
      }
    }
    const positions = new Float32Array(accepted);
    let depthSum = 0;
    let minD = Infinity;
    let maxD = -Infinity;
    for (let i = 0; i < positions.length; i += 3) {
      const z = positions[i + 2];
      depthSum += z;
      if (z < minD) minD = z;
      if (z > maxD) maxD = z;
    }
    const landmarkCount = positions.length / 3;
    return {
      frameIdx: frame.data.index,
      positions,
      landmarkCount,
      trackedCount,
      detectedCount: N,
      meanDepth: landmarkCount ? depthSum / landmarkCount : NaN,
      minDepth: landmarkCount ? minD : NaN,
      maxDepth: landmarkCount ? maxD : NaN,
      rawPoints: points,
    };
  }, [features.data, tri.data, frame.data, left, right, params]);

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
        id: 'init-landmarks',
        label: `초기 MapPoint ≥ num_features_init (${params.numFeaturesInit})`,
        pass: result.landmarkCount >= params.numFeaturesInit,
        detail: `landmarks = ${result.landmarkCount}, tracked = ${result.trackedCount} / ${result.detectedCount}`,
      });
      items.push({
        id: 'depth-positive',
        label: '모든 MapPoint depth > 0 (정면)',
        pass: result.landmarkCount > 0 && result.minDepth > 0,
        detail: Number.isFinite(result.minDepth) ? `min depth = ${result.minDepth.toFixed(2)} m` : undefined,
      });
      items.push({
        id: 'depth-spread',
        label: 'depth 범위가 1–80 m (KITTI 합리적)',
        pass:
          result.landmarkCount > 0 &&
          result.minDepth >= 1 &&
          result.maxDepth <= 80,
        detail: Number.isFinite(result.minDepth)
          ? `min ${result.minDepth.toFixed(2)} m · max ${result.maxDepth.toFixed(2)} m · mean ${result.meanDepth.toFixed(2)} m`
          : undefined,
      });
    } else {
      items.push({ id: 'init-landmarks', label: '초기 MapPoint ≥ num_features_init', pass: false, detail: '계산 대기' });
      items.push({ id: 'depth-positive', label: '모든 MapPoint depth > 0', pass: false });
      items.push({ id: 'depth-spread', label: 'depth 범위 1–80 m', pass: false });
    }
    return items;
  }, [features.data, tri.data, frame.data, left, right, result, params]);

  const frustums: CameraFrustum[] = useMemo(() => {
    if (!result || result.landmarkCount === 0) return [];
    return [
      // Left camera frustum at world origin (KeyFrame 0).
      {
        worldFromCamera: [
          1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1,
        ],
        scale: 1.5,
        color: [0.5, 0.95, 0.6],
        label: 'KF0 (left)',
      },
      // Right camera frustum at +baseline along x.
      ...(right ? [{
        worldFromCamera: [
          1, 0, 0, -right.t[0],
          0, 1, 0, -right.t[1],
          0, 0, 1, -right.t[2],
          0, 0, 0, 1,
        ],
        scale: 1.2,
        color: [0.4, 0.6, 1.0] as [number, number, number],
        label: 'right',
      }] : []),
    ];
  }, [result, right]);

  const pointCloud: PointCloudInput | null = useMemo(() => {
    if (!result || result.landmarkCount === 0) return null;
    return { positions: result.positions, size: 0.18 };
  }, [result]);

  const sceneOk = !!result && result.landmarkCount > 0;

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
          result={result}
          left={left}
          right={right}
        />
      }
      output={
        <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
          <KeyFrameCard result={result} cameraR={right} />
          {sceneOk ? (
            <Scene3D width={520} height={380} frustums={frustums} pointCloud={pointCloud} initialDistance={20} />
          ) : (
            <div style={{ fontSize: 12, color: 'var(--color-fg-faint)' }}>대기 중…</div>
          )}
        </div>
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
  params: Step6Params;
  setParams: (next: Step6Params) => void;
}) {
  const update = <K extends keyof Step6Params>(key: K, value: Step6Params[K]) =>
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
      <h3 style={sectionTitle}>BuildInitMap</h3>
      <Slider label="frame" value={frameIndex} min={0} max={Math.max(0, frameCount - 1)} step={1} onChange={setFrameIndex} format={(n) => String(n)} />
      <Slider label="num_features (GFTT seed)" value={params.maxFeatures} min={50} max={500} step={10} onChange={(n) => update('maxFeatures', n)} format={(n) => String(n)} />
      <Slider label="num_features_init (gate)" value={params.numFeaturesInit} min={10} max={200} step={5} onChange={(n) => update('numFeaturesInit', n)} format={(n) => String(n)} />
      <Slider label="quality threshold (σ4/σ3)" value={params.qualityThreshold} min={1e-4} max={0.5} step={1e-4} onChange={(n) => update('qualityThreshold', n)} format={(n) => n.toExponential(1)} />
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
        ch13 Frontend::BuildInitMap: 첫 프레임에서 GFTT → LK → 삼각화 결과로 KeyFrame 0과 초기 MapPoint를 생성. landmark 수 ≥ <code>num_features_init</code>(기본 50)이면 status가 INITING → TRACKING_GOOD으로 전이.
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
  result,
  left,
  right,
}: {
  result: InitMapResult | null;
  left: KittiCamera | null;
  right: KittiCamera | null;
}) {
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      <table style={tableStyle}>
        <tbody>
          <tr><th style={thStyle}>frame index</th><td style={tdStyle}>#{result?.frameIdx ?? '—'}</td></tr>
          <tr><th style={thStyle}>detected (left, GFTT)</th><td style={tdStyle}>{result?.detectedCount ?? '—'}</td></tr>
          <tr><th style={thStyle}>tracked (right, LK)</th><td style={tdStyle}>{result?.trackedCount ?? '—'}</td></tr>
          <tr><th style={thStyle}>baseline</th><td style={tdStyle}>{right ? `${right.baseline.toFixed(4)} m` : '—'}</td></tr>
          <tr><th style={thStyle}>fx (left, downsampled)</th><td style={tdStyle}>{left ? left.fx.toFixed(2) : '—'}</td></tr>
        </tbody>
      </table>
      <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
        Step 1~5의 파이프라인을 그대로 재사용합니다 (Dataset → GFTT → LK → DLT 삼각화). Step 6은 그 결과로 첫 KeyFrame과 초기 MapPoint 집합을 만들고 ch13의 INITING → TRACKING_GOOD 전이를 게이트합니다.
      </div>
    </div>
  );
}

function KeyFrameCard({
  result,
  cameraR,
}: {
  result: InitMapResult | null;
  cameraR: KittiCamera | null;
}) {
  const ok = !!result && result.landmarkCount > 0;
  return (
    <section
      style={{
        border: ok ? '1px solid #3a6' : '1px solid #444',
        borderRadius: 4,
        padding: 10,
        background: ok ? '#0f2017' : '#181818',
        fontSize: 13,
        color: '#ddd',
        display: 'flex',
        flexDirection: 'column',
        gap: 4,
      }}
    >
      <div style={{ display: 'flex', justifyContent: 'space-between' }}>
        <strong>{ok ? '✅ KeyFrame 0' : '⬜ KeyFrame 미생성'}</strong>
        <span style={{ color: '#888' }}>frame #{result?.frameIdx ?? '—'}</span>
      </div>
      {result && (
        <>
          <div>landmarks = <strong>{result.landmarkCount}</strong></div>
          <div>
            depth · min {Number.isFinite(result.minDepth) ? `${result.minDepth.toFixed(2)} m` : '—'}
            {' '}/ mean {Number.isFinite(result.meanDepth) ? `${result.meanDepth.toFixed(2)} m` : '—'}
            {' '}/ max {Number.isFinite(result.maxDepth) ? `${result.maxDepth.toFixed(2)} m` : '—'}
          </div>
          {cameraR && <div style={{ color: '#888' }}>baseline = {cameraR.baseline.toFixed(4)} m</div>}
        </>
      )}
    </section>
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

const tableStyle: React.CSSProperties = {
  borderCollapse: 'collapse',
  fontSize: 12,
  color: '#ddd',
};

const thStyle: React.CSSProperties = {
  textAlign: 'left',
  color: '#888',
  fontWeight: 'normal',
  padding: '3px 8px 3px 0',
  borderBottom: '1px solid #222',
};

const tdStyle: React.CSSProperties = {
  padding: '3px 0',
  borderBottom: '1px solid #222',
};
