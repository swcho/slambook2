import { useEffect, useMemo, useRef, useState } from 'react';
import { useQuery } from '@tanstack/react-query';
import { STEPS } from '../index';
import { StepLayout } from '../../components/StepLayout';
import type { VerifyItem } from '../../components/VerifyGate';
import {
  formatMatrix3,
  loadKittiCalibFor,
  loadKittiFrame,
  type KittiCamera,
  type StereoFrame,
} from '../../lib/kitti';
import { useDataset } from '../../lib/useDataset';
import type { DatasetDef } from '../../lib/datasets';

const DOWNSAMPLE_OPTIONS = [0.25, 0.5, 1.0] as const;

function useCameras(dataset: DatasetDef, downsample: number) {
  return useQuery({
    queryKey: ['kitti', 'calib', dataset.id, downsample],
    queryFn: () => loadKittiCalibFor(dataset, downsample),
  });
}

function useFrame(dataset: DatasetDef, index: number, downsample: number) {
  return useQuery({
    queryKey: ['kitti', 'frame', dataset.id, index, downsample],
    queryFn: () => loadKittiFrame(dataset.dir, index, downsample),
  });
}

export function Step01Dataset() {
  const step = STEPS.find((s) => s.id === 1)!;
  const { dataset } = useDataset();
  const frameCount = dataset.frameCount;
  const [frameIndex, setFrameIndex] = useState(0);
  const [downsample, setDownsample] = useState<(typeof DOWNSAMPLE_OPTIONS)[number]>(0.5);

  const clampedIndex = Math.min(frameIndex, frameCount - 1);
  const cameras = useCameras(dataset, downsample);
  const frame = useFrame(dataset, clampedIndex, downsample);

  const verifyItems: VerifyItem[] = useMemo(() => {
    const cams = cameras.data ?? null;
    const fr = frame.data ?? null;
    const p1 = cams?.find((c) => c.id === 1);
    const items: VerifyItem[] = [];
    items.push({
      id: 'calib-parsed',
      label: '4개의 Projection matrix (P0~P3)가 파싱됨',
      pass: cams?.length === 4,
      detail: cams ? `파싱된 카메라 수: ${cams.length}` : cameras.isLoading ? '로딩 중…' : String(cameras.error),
    });
    items.push({
      id: 'baseline-positive',
      label: 'P1 baseline > 0 (스테레오 구성 확인)',
      pass: !!p1 && p1.baseline > 0,
      detail: p1 ? `baseline = ${p1.baseline.toFixed(4)} m` : undefined,
    });
    items.push({
      id: 'frame-loaded',
      label: '좌/우 이미지 로드 성공',
      pass: !!fr,
      detail: fr ? `${fr.width}×${fr.height}` : frame.isLoading ? '로딩 중…' : frame.error ? String(frame.error) : undefined,
    });
    items.push({
      id: 'stereo-dims-match',
      label: '좌/우 해상도 일치',
      pass: !!fr && fr.left.width === fr.right.width && fr.left.height === fr.right.height,
    });
    const expectedFx = p1 ? p1.P[0] * downsample : 0;
    items.push({
      id: 'downsample-applied',
      label: 'K 행렬에 downsample 계수 적용',
      pass: !!p1 && Math.abs(p1.fx - expectedFx) < 1e-3,
      detail: p1 ? `fx = ${p1.fx.toFixed(3)} (expected ${expectedFx.toFixed(3)})` : undefined,
    });
    return items;
  }, [cameras.data, cameras.isLoading, cameras.error, frame.data, frame.isLoading, frame.error, downsample]);

  return (
    <StepLayout
      step={step}
      paramPanel={
        <ParamPanel
          frameIndex={clampedIndex}
          setFrameIndex={setFrameIndex}
          frameCount={frameCount}
          downsample={downsample}
          setDownsample={setDownsample}
          datasetDir={dataset.dir}
        />
      }
      input={
        <StereoView
          frame={frame.data ?? null}
          loading={frame.isLoading || frame.isFetching}
          error={frame.error ? String(frame.error) : null}
        />
      }
      output={
        <CalibView
          cameras={cameras.data ?? null}
          frame={frame.data ?? null}
          downsample={downsample}
          error={cameras.error ? String(cameras.error) : null}
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
  downsample,
  setDownsample,
  datasetDir,
}: {
  frameIndex: number;
  setFrameIndex: (n: number) => void;
  frameCount: number;
  downsample: (typeof DOWNSAMPLE_OPTIONS)[number];
  setDownsample: (n: (typeof DOWNSAMPLE_OPTIONS)[number]) => void;
  datasetDir: string;
}) {
  return (
    <section
      style={{
        border: '1px solid #333',
        borderRadius: 6,
        padding: 12,
        background: '#181818',
        display: 'flex',
        flexDirection: 'column',
        gap: 12,
      }}
    >
      <h3 style={sectionTitle}>Parameters</h3>
      <label style={labelStyle}>
        <span>
          start frame: <strong>{frameIndex}</strong> / {Math.max(0, frameCount - 1)}
        </span>
        <input
          type="range"
          min={0}
          max={Math.max(0, frameCount - 1)}
          step={1}
          value={frameIndex}
          onChange={(e) => setFrameIndex(Number(e.target.value))}
        />
      </label>
      <div style={labelStyle}>
        <span>downsample</span>
        <div style={{ display: 'flex', gap: 8 }}>
          {DOWNSAMPLE_OPTIONS.map((opt) => (
            <button
              key={opt}
              type="button"
              onClick={() => setDownsample(opt)}
              style={{
                padding: '4px 10px',
                borderRadius: 4,
                border: '1px solid #444',
                background: downsample === opt ? '#2a3d5c' : '#222',
                color: '#eee',
                cursor: 'pointer',
                fontSize: 13,
              }}
            >
              {opt}×
            </button>
          ))}
        </div>
      </div>
      <div style={{ fontSize: 12, color: '#888', lineHeight: 1.6 }}>
        <div>
          dataset: <code>{datasetDir}</code>
        </div>
        <div>ch13/src/dataset.cpp는 고정 0.5× 적용. 학습 목적으로 가변 허용.</div>
      </div>
    </section>
  );
}

function StereoView({
  frame,
  loading,
  error,
}: {
  frame: StereoFrame | null;
  loading: boolean;
  error: string | null;
}) {
  const leftRef = useRef<HTMLCanvasElement | null>(null);
  const rightRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    if (!frame) return;
    drawImageData(leftRef.current, frame.left);
    drawImageData(rightRef.current, frame.right);
  }, [frame]);

  if (error) return <div style={{ color: '#f88', fontSize: 13 }}>{error}</div>;
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      <div style={{ fontSize: 12, color: '#888' }}>
        {loading
          ? 'loading…'
          : frame
          ? `frame #${frame.index} · ${frame.width}×${frame.height}`
          : '—'}
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

function CalibView({
  cameras,
  frame,
  downsample,
  error,
}: {
  cameras: KittiCamera[] | null;
  frame: StereoFrame | null;
  downsample: number;
  error: string | null;
}) {
  if (error) return <div style={{ color: '#f88', fontSize: 13 }}>{error}</div>;
  if (!cameras) return <div style={{ color: '#888' }}>loading calib.txt…</div>;
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 12 }}>
      <div style={{ fontSize: 12, color: '#888' }}>
        4 cameras · downsample = {downsample}× · image size ={' '}
        {frame ? `${frame.width}×${frame.height}` : '—'}
      </div>
      {cameras.map((cam) => (
        <section
          key={cam.id}
          style={{
            border: '1px solid #2b2b2b',
            borderRadius: 4,
            padding: 10,
            background: '#141414',
          }}
        >
          <div style={{ display: 'flex', justifyContent: 'space-between', gap: 8, marginBottom: 6 }}>
            <strong style={{ fontSize: 13 }}>
              P{cam.id}{' '}
              {cam.id === 0
                ? '(left gray)'
                : cam.id === 1
                ? '(right gray)'
                : cam.id === 2
                ? '(left color)'
                : '(right color)'}
            </strong>
            <span style={{ color: '#9cf', fontSize: 12 }}>
              baseline = {cam.baseline.toFixed(4)} m
            </span>
          </div>
          <div style={{ fontSize: 12, color: '#bbb', display: 'flex', flexDirection: 'column', gap: 4 }}>
            <div>
              fx={cam.fx.toFixed(3)} &nbsp; fy={cam.fy.toFixed(3)} &nbsp;
              cx={cam.cx.toFixed(3)} &nbsp; cy={cam.cy.toFixed(3)}
            </div>
            <div>t = [ {cam.t.map((v) => v.toFixed(4)).join(', ')} ]</div>
            <pre style={matrixStyle}>
{`K =\n` + formatMatrix3(cam.K).join('\n')}
            </pre>
          </div>
        </section>
      ))}
    </div>
  );
}

function drawImageData(canvas: HTMLCanvasElement | null, data: ImageData) {
  if (!canvas) return;
  canvas.width = data.width;
  canvas.height = data.height;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  ctx.putImageData(data, 0, 0);
}

const sectionTitle: React.CSSProperties = {
  margin: '0 0 8px',
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

const figureStyle: React.CSSProperties = {
  margin: 0,
  display: 'flex',
  flexDirection: 'column',
  gap: 2,
};

const figCaptionStyle: React.CSSProperties = {
  fontSize: 11,
  color: '#888',
};

const canvasStyle: React.CSSProperties = {
  width: '100%',
  height: 'auto',
  border: '1px solid #333',
  background: '#000',
  imageRendering: 'pixelated',
};

const matrixStyle: React.CSSProperties = {
  margin: 0,
  fontFamily: 'ui-monospace, SF Mono, Menlo, monospace',
  fontSize: 11,
  color: '#cde',
  background: '#0c0c0c',
  padding: 6,
  borderRadius: 3,
  overflowX: 'auto',
};
