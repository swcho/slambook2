import { useMemo, useState } from 'react';
import { STEPS } from '../index';
import { StepLayout } from '../../components/StepLayout';
import type { VerifyItem } from '../../components/VerifyGate';
import { Scene3D, type CameraFrustum, type PointCloudInput } from '../../components/Scene3D';
import {
  SlamMap,
  buildSyntheticKfStream,
  type EvictionRecord,
  type Policy,
  type SyntheticKfStream,
} from '../../lib/slam/map';
import { se3Invert, type Mat4 } from '../../lib/slam/se3';

// Step 12 mirrors ch13/src/map.cpp::RemoveOldKeyframe. PLAN §3 Step 12 calls
// for at least 4 candidate policies; we expose all four and replay the same
// synthetic 10-KF stream under each so the user can compare evictions and
// post-eviction pose spread side-by-side.
//
// Why synthetic instead of real KITTI mini? mini fixture is 5 frames with
// tiny inter-frame motion (forward shift only), so every policy degenerates
// to "all KFs are duplicates" and the comparison loses educational value.
// The synthetic stream injects a duplicate cluster + diversity stretch so
// the ch13 default's branch decision visibly differs from FIFO and
// covisibility. Real-frame evaluation lives in Step 13 where the full VO
// loop drives KF insertion.

interface Step12Params {
  windowSize: number; // num_active_keyframes
  policy: Policy;
  duplicateDistanceThreshold: number; // ch13 hard-codes 0.2
  numForwardA: number;
  numDuplicates: number;
  numForwardB: number;
}

const DEFAULT_PARAMS: Step12Params = {
  windowSize: 4,
  policy: 'ch13-default',
  duplicateDistanceThreshold: 0.2,
  numForwardA: 5,
  numDuplicates: 2,
  numForwardB: 3,
};

const POLICY_OPTIONS: { value: Policy; label: string }[] = [
  { value: 'ch13-default', label: 'ch13 default (closest, then farthest)' },
  { value: 'fifo', label: 'FIFO (oldest insertion)' },
  { value: 'covisibility', label: 'covisibility (fewest shared landmarks)' },
  { value: 'distance-only', label: 'distance-only (always closest)' },
];

interface Insertion {
  step: number;
  kfFrameId: number;
  insertedKfId: number;
  eviction: EvictionRecord | null;
  activeIdsAfter: number[];
}

interface PolicyRunResult {
  policy: Policy;
  insertions: Insertion[];
  finalActiveKfIds: number[];
  finalActiveLandmarkIds: number[];
  poseSpreadVariance: number;
  totalEvictions: number;
  totalLandmarksRemoved: number;
}

interface AggregateResult {
  stream: SyntheticKfStream;
  perPolicy: Record<Policy, PolicyRunResult>;
}

export function Step12SlidingWindow() {
  const step = STEPS.find((s) => s.id === 12)!;
  const [params, setParams] = useState<Step12Params>(DEFAULT_PARAMS);

  const aggregate = useMemo<AggregateResult>(() => {
    const stream = buildSyntheticKfStream(
      params.numForwardA,
      params.numDuplicates,
      params.numForwardB,
    );
    const policies: Policy[] = ['ch13-default', 'fifo', 'covisibility', 'distance-only'];
    const perPolicy = {} as Record<Policy, PolicyRunResult>;
    for (const policy of policies) {
      const map = new SlamMap();
      map.numActiveKeyframes = params.windowSize;
      // Pre-create landmarks so KF observations link to the same ids across
      // policies. Order preserved by SlamMap's insertion counter.
      for (const lp of stream.landmarkPositions) {
        map.insertMapPoint(lp);
      }
      const insertions: Insertion[] = [];
      for (let i = 0; i < stream.poses.length; i++) {
        const { kf, eviction } = map.insertKeyframe(
          i,
          stream.poses[i],
          stream.observedLandmarkIds[i],
          policy,
          { duplicateDistanceThreshold: params.duplicateDistanceThreshold },
        );
        insertions.push({
          step: i,
          kfFrameId: i,
          insertedKfId: kf.id,
          eviction,
          activeIdsAfter: [...map.activeKeyframeIds],
        });
      }
      const spread = map.activePoseSpread();
      perPolicy[policy] = {
        policy,
        insertions,
        finalActiveKfIds: [...map.activeKeyframeIds],
        finalActiveLandmarkIds: [...map.activeLandmarkIds],
        poseSpreadVariance: spread.variance,
        totalEvictions: insertions.filter((ins) => ins.eviction).length,
        totalLandmarksRemoved: insertions.reduce(
          (acc, ins) => acc + (ins.eviction?.landmarksRemoved ?? 0),
          0,
        ),
      };
    }
    return { stream, perPolicy };
  }, [params]);

  const result = aggregate.perPolicy[params.policy];

  const verifyItems: VerifyItem[] = useMemo(() => {
    const items: VerifyItem[] = [];
    const totalKfs = aggregate.stream.poses.length;
    items.push({
      id: 'stream-built',
      label: 'synthetic KF stream 생성',
      pass: totalKfs >= params.windowSize + 1,
      detail: `${totalKfs} KF candidates · window ${params.windowSize}`,
    });
    items.push({
      id: 'eviction-fired',
      label: '활성 정책이 최소 1회 KF를 evict',
      pass: result.totalEvictions > 0,
      detail: `${result.totalEvictions} evictions across ${totalKfs} insertions`,
    });
    const expectedEvictions = Math.max(0, totalKfs - params.windowSize);
    items.push({
      id: 'eviction-count',
      label: `evict 횟수 = ${expectedEvictions} (총 KF − window)`,
      pass: result.totalEvictions === expectedEvictions,
      detail: `expected ${expectedEvictions}, got ${result.totalEvictions}`,
    });
    items.push({
      id: 'active-landmarks',
      label: '활성 landmark > 0 (cleanMap 후)',
      pass: result.finalActiveLandmarkIds.length > 0,
      detail: `${result.finalActiveLandmarkIds.length} active landmarks`,
    });
    items.push({
      id: 'all-policies',
      label: '모든 4 정책이 NaN 없이 종료',
      pass: Object.values(aggregate.perPolicy).every(
        (r) => Number.isFinite(r.poseSpreadVariance),
      ),
    });
    return items;
  }, [aggregate, result, params.windowSize]);

  return (
    <StepLayout
      step={step}
      paramPanel={<ParamPanel params={params} setParams={setParams} />}
      input={<InputView stream={aggregate.stream} />}
      output={
        <OutputView
          aggregate={aggregate}
          activePolicy={params.policy}
          windowSize={params.windowSize}
        />
      }
      verifyItems={verifyItems}
    />
  );
}

function ParamPanel({
  params,
  setParams,
}: {
  params: Step12Params;
  setParams: (next: Step12Params) => void;
}) {
  const update = <K extends keyof Step12Params>(key: K, value: Step12Params[K]) =>
    setParams({ ...params, [key]: value });

  return (
    <section style={paramPanelStyle}>
      <h3 style={sectionTitle}>Active window</h3>
      <Slider
        label="num_active_keyframes"
        value={params.windowSize}
        min={2}
        max={Math.max(2, params.numForwardA + params.numDuplicates + params.numForwardB - 1)}
        step={1}
        onChange={(n) => update('windowSize', n)}
        format={(n) => `${n} KFs (ch13 default 7)`}
      />

      <h3 style={sectionTitle}>Policy</h3>
      <div style={{ display: 'flex', flexDirection: 'column', gap: 4 }}>
        {POLICY_OPTIONS.map((opt) => (
          <button
            key={opt.value}
            type="button"
            onClick={() => update('policy', opt.value)}
            style={{
              ...algoBtn,
              textAlign: 'left',
              background: params.policy === opt.value ? '#2a3d5c' : '#1e1e1e',
            }}
          >
            {opt.label}
          </button>
        ))}
      </div>
      {params.policy === 'ch13-default' && (
        <Slider
          label="duplicate threshold"
          value={params.duplicateDistanceThreshold}
          min={0.01}
          max={2.0}
          step={0.01}
          onChange={(n) => update('duplicateDistanceThreshold', n)}
          format={(n) => `${n.toFixed(2)} m (ch13 = 0.20)`}
        />
      )}
      {params.policy !== 'ch13-default' && (
        <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
          duplicate threshold은 ch13 default 정책에서만 사용됩니다.
        </div>
      )}

      <h3 style={sectionTitle}>Stream synthesis</h3>
      <Slider
        label="forward A"
        value={params.numForwardA}
        min={2}
        max={8}
        step={1}
        onChange={(n) => update('numForwardA', n)}
        format={(n) => `${n} KFs`}
      />
      <Slider
        label="duplicates"
        value={params.numDuplicates}
        min={0}
        max={5}
        step={1}
        onChange={(n) => update('numDuplicates', n)}
        format={(n) => `${n} near KF#${params.numForwardA - 1}`}
      />
      <Slider
        label="forward B"
        value={params.numForwardB}
        min={0}
        max={6}
        step={1}
        onChange={(n) => update('numForwardB', n)}
        format={(n) => `${n} KFs`}
      />

      <button type="button" onClick={() => setParams(DEFAULT_PARAMS)} style={resetBtn}>
        reset to ch13 defaults
      </button>
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

function InputView({ stream }: { stream: SyntheticKfStream }) {
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      <div style={{ fontSize: 12, color: '#888', lineHeight: 1.5 }}>
        합성 KF 스트림: <strong>{stream.poses.length}</strong> KF 후보 ·{' '}
        <strong>{stream.numLandmarks}</strong> landmarks
      </div>
      <table style={tableStyle}>
        <thead>
          <tr style={{ borderBottom: '1px solid #333', color: '#888' }}>
            <th style={cellL}>insert step</th>
            <th style={cellR}>tx</th>
            <th style={cellR}>tz</th>
            <th style={cellR}>observed lm</th>
          </tr>
        </thead>
        <tbody>
          {stream.poses.map((pose, i) => {
            const Twc = se3Invert(pose);
            return (
              <tr key={i} style={{ borderBottom: '1px solid #222' }}>
                <td style={cellL}>{i}</td>
                <td style={cellR}>{Twc[3].toFixed(2)}</td>
                <td style={cellR}>{Twc[11].toFixed(2)}</td>
                <td style={cellR}>{stream.observedLandmarkIds[i].size}</td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}

function OutputView({
  aggregate,
  activePolicy,
  windowSize,
}: {
  aggregate: AggregateResult;
  activePolicy: Policy;
  windowSize: number;
}) {
  const result = aggregate.perPolicy[activePolicy];
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 12 }}>
      <PolicyComparison aggregate={aggregate} active={activePolicy} />
      <EvictionLog insertions={result.insertions} />
      <Scene
        stream={aggregate.stream}
        result={result}
        windowSize={windowSize}
      />
    </div>
  );
}

function PolicyComparison({
  aggregate,
  active,
}: {
  aggregate: AggregateResult;
  active: Policy;
}) {
  return (
    <table style={tableStyle}>
      <thead>
        <tr style={{ borderBottom: '1px solid #333', color: '#888' }}>
          <th style={cellL}>policy</th>
          <th style={cellR}>evictions</th>
          <th style={cellR}>landmarks dropped</th>
          <th style={cellR}>active KFs</th>
          <th style={cellR}>active LMs</th>
          <th style={cellR}>pose spread Var</th>
        </tr>
      </thead>
      <tbody>
        {POLICY_OPTIONS.map(({ value, label }) => {
          const r = aggregate.perPolicy[value];
          const highlighted = value === active;
          return (
            <tr
              key={value}
              style={{
                borderBottom: '1px solid #222',
                background: highlighted ? '#1e2c44' : 'transparent',
              }}
            >
              <td style={cellL}>{label}</td>
              <td style={cellR}>{r.totalEvictions}</td>
              <td style={cellR}>{r.totalLandmarksRemoved}</td>
              <td style={cellR}>{r.finalActiveKfIds.length}</td>
              <td style={cellR}>{r.finalActiveLandmarkIds.length}</td>
              <td style={cellR}>{r.poseSpreadVariance.toFixed(3)}</td>
            </tr>
          );
        })}
      </tbody>
    </table>
  );
}

function EvictionLog({ insertions }: { insertions: Insertion[] }) {
  return (
    <figure style={{ margin: 0 }}>
      <figcaption style={figCaption}>insertion + eviction history</figcaption>
      <table style={tableStyle}>
        <thead>
          <tr style={{ borderBottom: '1px solid #333', color: '#888' }}>
            <th style={cellL}>step</th>
            <th style={cellL}>inserted KF</th>
            <th style={cellL}>active</th>
            <th style={cellL}>eviction</th>
          </tr>
        </thead>
        <tbody>
          {insertions.map((ins) => {
            const evicted = ins.eviction;
            return (
              <tr
                key={ins.step}
                style={{
                  borderBottom: '1px solid #222',
                  background: evicted ? '#22141a' : 'transparent',
                }}
              >
                <td style={cellL}>{ins.step}</td>
                <td style={cellL}>KF#{ins.insertedKfId}</td>
                <td style={cellL}>
                  <span style={{ color: '#aaa', fontSize: 11 }}>
                    [{ins.activeIdsAfter.map((id) => `#${id}`).join(', ')}]
                  </span>
                </td>
                <td style={cellL}>
                  {evicted ? (
                    <span style={{ color: '#f99' }}>
                      ✗ KF#{evicted.evictedKfId} — {evicted.reason}
                      {evicted.landmarksRemoved > 0 ? ` (−${evicted.landmarksRemoved} lm)` : null}
                    </span>
                  ) : (
                    <span style={{ color: 'var(--color-fg-faint)' }}>—</span>
                  )}
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </figure>
  );
}

function Scene({
  stream,
  result,
  windowSize,
}: {
  stream: SyntheticKfStream;
  result: PolicyRunResult;
  windowSize: number;
}) {
  const { frustums, pointCloud } = useMemo(() => {
    const frustums: CameraFrustum[] = [];
    const activeIds = new Set(result.finalActiveKfIds);
    // Stream order: insert step i → KF id == i (matches SlamMap counter as
    // long as we created landmarks first — they consumed the mp counter, not
    // the kf counter).
    for (let i = 0; i < stream.poses.length; i++) {
      const Twc = se3Invert(stream.poses[i]) as Mat4;
      const isActive = activeIds.has(i);
      // Active = cyan; evicted = dim gray.
      const color: [number, number, number] = isActive ? [0.4, 0.85, 1.0] : [0.4, 0.4, 0.4];
      frustums.push({
        worldFromCamera: Array.from(Twc),
        scale: isActive ? 0.6 : 0.4,
        color,
        label: `KF#${i}${isActive ? '' : '✗'}`,
      });
    }
    // Landmark cloud: split into active (highlighted) vs inactive (dim).
    const activeLm = new Set(result.finalActiveLandmarkIds);
    const positions = new Float32Array(stream.numLandmarks * 3);
    const colors = new Float32Array(stream.numLandmarks * 3);
    for (let i = 0; i < stream.numLandmarks; i++) {
      positions[3 * i + 0] = stream.landmarkPositions[i][0];
      positions[3 * i + 1] = stream.landmarkPositions[i][1];
      positions[3 * i + 2] = stream.landmarkPositions[i][2];
      const isActive = activeLm.has(i);
      colors[3 * i + 0] = isActive ? 0.95 : 0.35;
      colors[3 * i + 1] = isActive ? 0.85 : 0.35;
      colors[3 * i + 2] = isActive ? 0.25 : 0.35;
    }
    const pointCloud: PointCloudInput = { positions, colors, size: 0.18 };
    return { frustums, pointCloud };
  }, [stream, result]);

  return (
    <div>
      <div style={{ fontSize: 12, color: '#aaa', marginBottom: 4 }}>
        밝은 시안 frustum + 노란 점 = active window {windowSize} KFs · 회색 = evicted/inactive
      </div>
      <Scene3D width={620} height={340} frustums={frustums} pointCloud={pointCloud} initialDistance={18} />
    </div>
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
  fontSize: 12,
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

const tableStyle: React.CSSProperties = {
  borderCollapse: 'collapse',
  fontSize: 11,
  color: '#ccc',
  width: '100%',
};

const cellL: React.CSSProperties = { textAlign: 'left', padding: '2px 4px' };
const cellR: React.CSSProperties = { textAlign: 'right', padding: '2px 4px' };

const figCaption: React.CSSProperties = { fontSize: 11, color: '#888' };

export { Step12SlidingWindow as default };
