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

interface DetectParams {
  detector: DetectorKey;
  maxFeatures: number;
  qualityLevel: number;
  minDistance: number;
  blockSize: number;
  fastThreshold: number;
  orbScaleFactor: number;
  orbNLevels: number;
}

const DEFAULT_PARAMS: DetectParams = {
  detector: 'GFTT',
  maxFeatures: 150, // ch13 default num_features
  qualityLevel: 0.01, // ch13 default
  minDistance: 20, // ch13 default
  blockSize: 3,
  fastThreshold: 20,
  orbScaleFactor: 1.2,
  orbNLevels: 8,
};

// 분포 균등성 게이지: 이미지를 4×4 그리드로 나눠 keypoint가 떨어진 셀 수.
// 게이트 임계는 ≥ 8셀 (절반 이상에 분포).
function gridCoverage(kps: Float64Array, w: number, h: number, gx = 4, gy = 4): number {
  const cw = w / gx;
  const ch = h / gy;
  const cells = new Set<number>();
  for (let i = 0; i < kps.length; i += 3) {
    const cx = Math.min(gx - 1, Math.floor(kps[i] / cw));
    const cy = Math.min(gy - 1, Math.floor(kps[i + 1] / ch));
    cells.add(cy * gx + cx);
  }
  return cells.size;
}

export function Step03FeatureDetection() {
  const step = STEPS.find((s) => s.id === 3)!;
  const { dataset } = useDataset();
  const frameCount = dataset.frameCount;
  const [frameIndex, setFrameIndex] = useState(0);
  const clampedFrame = Math.min(frameIndex, frameCount - 1);
  const [params, setParams] = useState<DetectParams>(DEFAULT_PARAMS);

  // We always load at the same downsample as Step 1's default (0.5×) so
  // pipeline stages stay consistent.
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

  // Keep the gray buffer of the left image cached so detect re-runs are cheap.
  const grayLeft = useMemo(() => {
    if (!frame.data) return null;
    return imageDataToGray(frame.data.left);
  }, [frame.data]);

  // Run detection whenever the inputs change. Wrap in useMemo so we only
  // re-detect when params or frame change.
  const detection = useMemo(() => {
    if (!wasm.data || !frame.data || !grayLeft) return null;
    const t0 = performance.now();
    const out = wasm.data.detect(
      grayLeft,
      frame.data.width,
      frame.data.height,
      params.detector,
      {
        maxFeatures: params.maxFeatures,
        qualityLevel: params.qualityLevel,
        minDistance: params.minDistance,
        blockSize: params.blockSize,
        fastThreshold: params.fastThreshold,
        orbScaleFactor: params.orbScaleFactor,
        orbNLevels: params.orbNLevels,
      },
    );
    const dt = performance.now() - t0;
    return { points: out, ms: dt };
  }, [wasm.data, frame.data, grayLeft, params]);

  const numKps = detection ? detection.points.length / 3 : 0;
  const coverage = useMemo(() => {
    if (!detection || !frame.data) return 0;
    return gridCoverage(detection.points, frame.data.width, frame.data.height);
  }, [detection, frame.data]);

  const verifyItems: VerifyItem[] = useMemo(() => {
    const items: VerifyItem[] = [];
    items.push({
      id: 'wasm-loaded',
      label: 'features WASM 모듈 로드 (OpenCV 4.13.0)',
      pass: !!wasm.data && wasm.data.opencvVersion === '4.13.0',
      detail: wasm.data
        ? `OpenCV ${wasm.data.opencvVersion}`
        : wasm.error
        ? String(wasm.error)
        : '로딩 중…',
    });
    items.push({
      id: 'frame-loaded',
      label: '좌측 이미지 로드',
      pass: !!frame.data,
      detail: frame.data ? `${frame.data.width}×${frame.data.height}` : undefined,
    });
    // Settled near the requested count: ≥ 50% of maxFeatures, or hard floor 30.
    const enoughKps = numKps >= Math.max(30, params.maxFeatures * 0.5);
    items.push({
      id: 'enough-kps',
      label: `검출 수가 설정값 근처 (≥ ${Math.max(30, Math.round(params.maxFeatures * 0.5))})`,
      pass: enoughKps,
      detail: `검출 ${numKps} / 요청 ${params.maxFeatures}`,
    });
    items.push({
      id: 'coverage',
      label: '4×4 그리드 중 ≥ 8 셀에 keypoint 분포',
      pass: coverage >= 8,
      detail: `${coverage} / 16 cells`,
    });
    return items;
  }, [wasm.data, wasm.error, frame.data, numKps, coverage, params.maxFeatures]);

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
          loading={frame.isLoading || frame.isFetching}
          calibInfo={
            cameras.data?.find((c) => c.id === 0)
              ? `K (P0, ${downsample}×): fx=${cameras.data.find((c) => c.id === 0)!.fx.toFixed(2)}`
              : undefined
          }
        />
      }
      output={
        <OutputView
          frame={frame.data ?? null}
          detection={detection}
          numKps={numKps}
          coverage={coverage}
          detector={params.detector}
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
  params: DetectParams;
  setParams: (next: DetectParams) => void;
}) {
  const update = <K extends keyof DetectParams>(key: K, value: DetectParams[K]) =>
    setParams({ ...params, [key]: value });
  const isGFTTLike = params.detector === 'GFTT' || params.detector === 'Harris';
  const isFAST = params.detector === 'FAST';
  const isORB = params.detector === 'ORB';
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

      <h3 style={sectionTitle}>Parameters</h3>
      <Slider
        label="frame"
        value={frameIndex}
        min={0}
        max={Math.max(0, frameCount - 1)}
        step={1}
        onChange={setFrameIndex}
        format={(n) => String(n)}
      />
      <Slider
        label="maxFeatures"
        value={params.maxFeatures}
        min={20}
        max={500}
        step={10}
        onChange={(n) => update('maxFeatures', n)}
        format={(n) => String(n)}
      />

      {isGFTTLike && (
        <>
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
            label="minDistance (px)"
            value={params.minDistance}
            min={3}
            max={50}
            step={1}
            onChange={(n) => update('minDistance', n)}
            format={(n) => String(n)}
          />
          <Slider
            label="blockSize"
            value={params.blockSize}
            min={3}
            max={11}
            step={2}
            onChange={(n) => update('blockSize', n)}
            format={(n) => String(n)}
          />
        </>
      )}
      {isFAST && (
        <Slider
          label="fastThreshold"
          value={params.fastThreshold}
          min={5}
          max={80}
          step={1}
          onChange={(n) => update('fastThreshold', n)}
          format={(n) => String(n)}
        />
      )}
      {isORB && (
        <>
          <Slider
            label="orbScaleFactor"
            value={params.orbScaleFactor}
            min={1.05}
            max={2.0}
            step={0.05}
            onChange={(n) => update('orbScaleFactor', n)}
            format={(n) => n.toFixed(2)}
          />
          <Slider
            label="orbNLevels"
            value={params.orbNLevels}
            min={2}
            max={12}
            step={1}
            onChange={(n) => update('orbNLevels', n)}
            format={(n) => String(n)}
          />
        </>
      )}
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
        ch13 원본은 GFTT, num_features=150, qualityLevel=0.01, minDistance=20.
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
      <input
        type="range"
        min={min}
        max={max}
        step={step}
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
      />
    </label>
  );
}

function InputView({
  frame,
  loading,
  calibInfo,
}: {
  frame: { width: number; height: number; left: ImageData; index: number } | null;
  loading: boolean;
  calibInfo?: string;
}) {
  const ref = useRef<HTMLCanvasElement | null>(null);
  useEffect(() => {
    if (!frame || !ref.current) return;
    ref.current.width = frame.width;
    ref.current.height = frame.height;
    const ctx = ref.current.getContext('2d');
    ctx?.putImageData(frame.left, 0, 0);
  }, [frame]);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      <div style={{ fontSize: 12, color: '#888' }}>
        {loading ? 'loading…' : frame ? `frame #${frame.index} · ${frame.width}×${frame.height} (left, gray)` : '—'}
      </div>
      <canvas ref={ref} style={canvasStyle} />
      {calibInfo && <div style={{ fontSize: 11, color: '#888' }}>{calibInfo}</div>}
    </div>
  );
}

function OutputView({
  frame,
  detection,
  numKps,
  coverage,
  detector,
}: {
  frame: { width: number; height: number; left: ImageData } | null;
  detection: { points: Float64Array; ms: number } | null;
  numKps: number;
  coverage: number;
  detector: DetectorKey;
}) {
  const ref = useRef<HTMLCanvasElement | null>(null);
  useEffect(() => {
    if (!frame || !ref.current) return;
    const canvas = ref.current;
    canvas.width = frame.width;
    canvas.height = frame.height;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    ctx.putImageData(frame.left, 0, 0);
    if (!detection) return;
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = '#33ff66';
    for (let i = 0; i < detection.points.length; i += 3) {
      const x = detection.points[i];
      const y = detection.points[i + 1];
      ctx.beginPath();
      ctx.arc(x, y, 3, 0, Math.PI * 2);
      ctx.stroke();
    }
  }, [frame, detection]);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      <div style={{ fontSize: 12, color: '#aaa' }}>
        algorithm: <strong>{detector}</strong>
        {detection ? (
          <>
            {' · '}
            <span>{numKps} keypoints</span>
            {' · '}
            <span>{detection.ms.toFixed(1)} ms</span>
            {' · '}
            <span>coverage {coverage}/16</span>
          </>
        ) : (
          <> · loading…</>
        )}
      </div>
      <canvas ref={ref} style={canvasStyle} />
      <div style={{ fontSize: 11, color: '#888' }}>
        ● 녹색 원: 검출된 keypoint. 분포가 한쪽으로 쏠리면 minDistance/qualityLevel 조정 필요.
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

const canvasStyle: React.CSSProperties = {
  width: '100%',
  height: 'auto',
  border: '1px solid #333',
  background: '#000',
  imageRendering: 'pixelated',
};
