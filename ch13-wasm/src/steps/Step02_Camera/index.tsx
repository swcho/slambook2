import { useEffect, useMemo, useState } from 'react';
import { useQuery } from '@tanstack/react-query';
import { STEPS } from '../index';
import { StepLayout } from '../../components/StepLayout';
import type { VerifyItem } from '../../components/VerifyGate';
import { loadCameraWasm, type CameraModule } from '../../wasm/camera';

// Defaults: KITTI 05 at downsample 0.5 (matches Step 1 output).
const DEFAULT_INTRINSICS = {
  fx: 353.5456,
  fy: 353.5456,
  cx: 300.94365,
  cy: 91.5552,
} as const;
const DEFAULT_BASELINE = 0.5371554;
const DEFAULT_POINT = { x: 1.0, y: 0.5, z: 12.0 };

// Fixed test grid for the round-trip verification. Range covers near/far + the
// four image quadrants so the gate catches bugs that only surface off-centre.
function buildTestGrid(): Float64Array {
  const zs = [3, 6, 12, 24];
  const xs = [-3, -1, 0, 1, 3];
  const ys = [-1.5, 0, 1.5];
  const pts: number[] = [];
  for (const z of zs) for (const y of ys) for (const x of xs) pts.push(x, y, z);
  return Float64Array.from(pts);
}
const TEST_GRID = buildTestGrid();
const ROUND_TRIP_THRESHOLD = 1e-5;

export function Step02Camera() {
  const step = STEPS.find((s) => s.id === 2)!;

  const { data: module, isLoading, error } = useQuery<CameraModule>({
    queryKey: ['wasm', 'camera', 'baseline'],
    queryFn: () => loadCameraWasm('baseline'),
  });

  const [fx, setFx] = useState<number>(DEFAULT_INTRINSICS.fx);
  const [fy, setFy] = useState<number>(DEFAULT_INTRINSICS.fy);
  const [cx, setCx] = useState<number>(DEFAULT_INTRINSICS.cx);
  const [cy, setCy] = useState<number>(DEFAULT_INTRINSICS.cy);
  const [camIdx, setCamIdx] = useState<0 | 1>(0);
  const [pt, setPt] = useState(DEFAULT_POINT);

  // Rebuild the WASM Camera handle whenever intrinsics/extrinsic change.
  // Delete the previous handle to keep the Emscripten heap tidy.
  const cam = useMemo(() => {
    if (!module) return null;
    const tx = camIdx === 0 ? 0 : -DEFAULT_BASELINE;
    return new module.Camera(fx, fy, cx, cy, DEFAULT_BASELINE, tx, 0, 0);
  }, [module, fx, fy, cx, cy, camIdx]);

  useEffect(() => {
    return () => {
      cam?.delete();
    };
  }, [cam]);

  const computed = useMemo(() => {
    if (!module || !cam) return null;
    const uv = cam.worldToPixel(pt.x, pt.y, pt.z);
    const xyz = cam.pixelToWorld(uv[0], uv[1], pt.z);
    const dx = xyz[0] - pt.x;
    const dy = xyz[1] - pt.y;
    const dz = xyz[2] - pt.z;
    const singleError = Math.hypot(dx, dy, dz);
    const gridError = module.roundTripMaxError(cam, TEST_GRID);
    return { uv, xyz, singleError, gridError };
  }, [module, cam, pt]);

  const verifyItems: VerifyItem[] = useMemo(() => {
    const items: VerifyItem[] = [];
    items.push({
      id: 'wasm-loaded',
      label: 'Camera WASM 모듈 로드',
      pass: !!module,
      detail: error ? String(error) : isLoading ? '로딩 중…' : 'loaded',
    });
    items.push({
      id: 'round-trip-grid',
      label: `${TEST_GRID.length / 3}개 test point round-trip 오차 < ${ROUND_TRIP_THRESHOLD}`,
      pass: !!computed && computed.gridError < ROUND_TRIP_THRESHOLD,
      detail: computed ? `max error = ${computed.gridError.toExponential(3)}` : undefined,
    });
    items.push({
      id: 'current-point-match',
      label: '현재 파라미터로 입력 점 round-trip 일치',
      pass: !!computed && computed.singleError < ROUND_TRIP_THRESHOLD,
      detail: computed ? `error = ${computed.singleError.toExponential(3)}` : undefined,
    });
    items.push({
      id: 'stereo-pair-baseline',
      label: '선택된 카메라의 extrinsic t가 baseline 크기와 일치',
      pass: !!cam && Math.abs(Math.abs(cam.tx) - (camIdx === 0 ? 0 : DEFAULT_BASELINE)) < 1e-9,
      detail: cam ? `t = [${cam.tx.toFixed(4)}, ${cam.ty.toFixed(4)}, ${cam.tz.toFixed(4)}]` : undefined,
    });
    return items;
  }, [module, error, isLoading, computed, cam, camIdx]);

  return (
    <StepLayout
      step={step}
      paramPanel={
        <ParamPanel
          fx={fx}
          fy={fy}
          cx={cx}
          cy={cy}
          camIdx={camIdx}
          pt={pt}
          setFx={setFx}
          setFy={setFy}
          setCx={setCx}
          setCy={setCy}
          setCamIdx={setCamIdx}
          setPt={setPt}
          resetIntrinsics={() => {
            setFx(DEFAULT_INTRINSICS.fx);
            setFy(DEFAULT_INTRINSICS.fy);
            setCx(DEFAULT_INTRINSICS.cx);
            setCy(DEFAULT_INTRINSICS.cy);
          }}
        />
      }
      input={<InputView fx={fx} fy={fy} cx={cx} cy={cy} camIdx={camIdx} pt={pt} />}
      output={<OutputView computed={computed} />}
      verifyItems={verifyItems}
    />
  );
}

function ParamPanel({
  fx,
  fy,
  cx,
  cy,
  camIdx,
  pt,
  setFx,
  setFy,
  setCx,
  setCy,
  setCamIdx,
  setPt,
  resetIntrinsics,
}: {
  fx: number;
  fy: number;
  cx: number;
  cy: number;
  camIdx: 0 | 1;
  pt: { x: number; y: number; z: number };
  setFx: (n: number) => void;
  setFy: (n: number) => void;
  setCx: (n: number) => void;
  setCy: (n: number) => void;
  setCamIdx: (i: 0 | 1) => void;
  setPt: (p: { x: number; y: number; z: number }) => void;
  resetIntrinsics: () => void;
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
        gap: 10,
      }}
    >
      <h3 style={sectionTitle}>Parameters</h3>

      <div style={labelStyle}>
        <span>camera</span>
        <div style={{ display: 'flex', gap: 8 }}>
          {[0, 1].map((i) => (
            <button
              key={i}
              type="button"
              onClick={() => setCamIdx(i as 0 | 1)}
              style={{
                padding: '4px 10px',
                borderRadius: 4,
                border: '1px solid #444',
                background: camIdx === i ? '#2a3d5c' : '#222',
                color: '#eee',
                cursor: 'pointer',
                fontSize: 13,
              }}
            >
              {i === 0 ? 'P0 (left)' : 'P1 (right)'}
            </button>
          ))}
        </div>
      </div>

      <Slider label="fx" value={fx} min={100} max={800} step={0.1} onChange={setFx} />
      <Slider label="fy" value={fy} min={100} max={800} step={0.1} onChange={setFy} />
      <Slider label="cx" value={cx} min={50} max={700} step={0.1} onChange={setCx} />
      <Slider label="cy" value={cy} min={20} max={300} step={0.1} onChange={setCy} />

      <button
        type="button"
        onClick={resetIntrinsics}
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
        reset to KITTI 05 (0.5×)
      </button>

      <div style={{ borderTop: '1px solid #333', paddingTop: 8, display: 'flex', flexDirection: 'column', gap: 6 }}>
        <div style={{ fontSize: 12, color: '#aaa' }}>test point (world, metres)</div>
        <Slider label="x" value={pt.x} min={-5} max={5} step={0.1} onChange={(v) => setPt({ ...pt, x: v })} />
        <Slider label="y" value={pt.y} min={-3} max={3} step={0.05} onChange={(v) => setPt({ ...pt, y: v })} />
        <Slider label="z" value={pt.z} min={1} max={50} step={0.1} onChange={(v) => setPt({ ...pt, z: v })} />
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
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  onChange: (n: number) => void;
}) {
  return (
    <label style={labelStyle}>
      <span style={{ display: 'flex', justifyContent: 'space-between' }}>
        <span>{label}</span>
        <strong>{value.toFixed(3)}</strong>
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
  fx,
  fy,
  cx,
  cy,
  camIdx,
  pt,
}: {
  fx: number;
  fy: number;
  cx: number;
  cy: number;
  camIdx: 0 | 1;
  pt: { x: number; y: number; z: number };
}) {
  const K = [
    [fx, 0, cx],
    [0, fy, cy],
    [0, 0, 1],
  ];
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 10 }}>
      <div style={{ fontSize: 12, color: '#aaa' }}>
        camera: <strong>{camIdx === 0 ? 'P0 (left)' : 'P1 (right)'}</strong>
      </div>
      <pre style={matrixStyle}>
{`K =\n${K.map((r) => r.map((v) => v.toFixed(3).padStart(10, ' ')).join(' ')).join('\n')}`}
      </pre>
      <div style={{ fontSize: 12, color: '#bbb' }}>
        world point p = ( {pt.x.toFixed(3)}, {pt.y.toFixed(3)}, {pt.z.toFixed(3)} ) &nbsp;[m]
      </div>
      <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
        Step 2는 T<sub>c/w</sub> = I 가정 하의 핀홀 투영만 다룹니다. SE(3) 포즈 통합은 Phase E(Step 7/8)에서 진행됩니다.
      </div>
    </div>
  );
}

function OutputView({
  computed,
}: {
  computed:
    | null
    | {
        uv: [number, number];
        xyz: [number, number, number];
        singleError: number;
        gridError: number;
      };
}) {
  if (!computed) return <div style={{ color: '#888' }}>loading WASM module…</div>;
  const { uv, xyz, singleError, gridError } = computed;
  const passSingle = singleError < ROUND_TRIP_THRESHOLD;
  const passGrid = gridError < ROUND_TRIP_THRESHOLD;
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 10, fontSize: 13, color: '#ddd' }}>
      <div>
        <div style={{ color: '#aaa', fontSize: 12 }}>worldToPixel(p)</div>
        <div>
          [ u, v ] = ( <strong>{uv[0].toFixed(3)}</strong>, <strong>{uv[1].toFixed(3)}</strong> )
        </div>
      </div>
      <div>
        <div style={{ color: '#aaa', fontSize: 12 }}>pixelToWorld(u, v, z)</div>
        <div>
          [ x, y, z ] = ( {xyz[0].toFixed(6)}, {xyz[1].toFixed(6)}, {xyz[2].toFixed(6)} )
        </div>
      </div>
      <div
        style={{
          border: '1px solid #2b2b2b',
          borderRadius: 4,
          padding: 8,
          background: '#141414',
          fontSize: 12,
          display: 'flex',
          flexDirection: 'column',
          gap: 4,
        }}
      >
        <div>
          <span style={{ color: passSingle ? '#6c6' : '#d77' }}>●</span>{' '}
          current-point round-trip error ={' '}
          <code>{singleError.toExponential(3)}</code>
          {passSingle ? ' (pass)' : ' (FAIL)'}
        </div>
        <div>
          <span style={{ color: passGrid ? '#6c6' : '#d77' }}>●</span>{' '}
          {TEST_GRID.length / 3}-point grid max error ={' '}
          <code>{gridError.toExponential(3)}</code>
          {passGrid ? ' (pass)' : ' (FAIL)'}
        </div>
      </div>
    </div>
  );
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
