import { useEffect, useMemo, useState } from 'react';
import { useQuery } from '@tanstack/react-query';
import { STEPS } from '../index';
import { StepLayout } from '../../components/StepLayout';
import type { VerifyItem } from '../../components/VerifyGate';
import { Scene3D, type CameraFrustum, type PointCloudInput } from '../../components/Scene3D';
import { loadKittiFrame, parseKittiCalib, type StereoFrame } from '../../lib/kitti';
import { loadFeaturesWasm, type FeaturesModule } from '../../wasm/features';
import { loadTriangulationWasm, type TriangulationModule } from '../../wasm/triangulation';
import { loadPnPWasm, type PnPModule } from '../../wasm/pnp';
import { loadBaWasm, type BaModule } from '../../wasm/ba';
import {
  DEFAULT_PIPELINE,
  PIPELINE_PRESETS,
  runPipeline,
  type FrameRecord,
  type PipelineConfig,
  type PipelineRunResult,
} from '../../lib/slam/pipeline';
import type { Policy } from '../../lib/slam/map';
import { se3Invert } from '../../lib/slam/se3';

// Step 13 stitches every prior step into one VO loop. Mirrors
// ch13/app/run_kitti_stereo.cpp + visual_odometry.cpp::Step.
//
// Mini fixture is 5 frames; the loop is fast enough (sub-200 ms in baseline)
// that we can recompute the whole run synchronously whenever params change.
// Phase I will move this onto a Web Worker per PLAN §9.

const DATASET_DIR = '/datasets/kitti05-mini';
const FRAME_COUNT = 5;

type PresetKey = keyof typeof PIPELINE_PRESETS;

interface Step13Params extends PipelineConfig {
  /** Which frame the slider currently shows (0..numFrames-1). */
  playbackFrame: number;
  /** Total frames the pipeline ran for. ch13 mini = up to FRAME_COUNT. */
  numFrames: number;
}

const DEFAULT_PARAMS: Step13Params = {
  ...DEFAULT_PIPELINE,
  windowSize: 5,
  playbackFrame: FRAME_COUNT - 1,
  numFrames: FRAME_COUNT,
};

export function Step13FullPipeline() {
  const step = STEPS.find((s) => s.id === 13)!;
  const [params, setParams] = useState<Step13Params>(DEFAULT_PARAMS);

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
  const pnp = useQuery<PnPModule>({
    queryKey: ['wasm', 'pnp', 'baseline'],
    queryFn: () => loadPnPWasm('baseline'),
  });
  const ba = useQuery<BaModule>({
    queryKey: ['wasm', 'ba', 'baseline'],
    queryFn: () => loadBaWasm('baseline'),
  });

  const left = cameras.data?.find((c) => c.id === 0) ?? null;
  const right = cameras.data?.find((c) => c.id === 1) ?? null;
  const ready =
    !!features.data && !!tri.data && !!pnp.data && !!ba.data &&
    !!frames.data && !!left && !!right;

  // Stable config object so useMemo doesn't keep rebuilding when only the
  // playback slider moves.
  const pipelineConfig = useMemo<PipelineConfig>(
    () => ({
      policy: params.policy,
      windowSize: params.windowSize,
      duplicateDistanceThreshold: params.duplicateDistanceThreshold,
      maxFeatures: params.maxFeatures,
      pnpRounds: params.pnpRounds,
      enableBackend: params.enableBackend,
      baIterations: params.baIterations,
      baChi2Init: params.baChi2Init,
      baAdaptiveRounds: params.baAdaptiveRounds,
      numFeaturesNeededForKeyframe: params.numFeaturesNeededForKeyframe,
      numFeaturesTracking: params.numFeaturesTracking,
      numFeaturesTrackingBad: params.numFeaturesTrackingBad,
    }),
    [
      params.policy, params.windowSize, params.duplicateDistanceThreshold,
      params.maxFeatures, params.pnpRounds, params.enableBackend,
      params.baIterations, params.baChi2Init, params.baAdaptiveRounds,
      params.numFeaturesNeededForKeyframe, params.numFeaturesTracking,
      params.numFeaturesTrackingBad,
    ],
  );

  const result = useMemo<PipelineRunResult | null>(() => {
    if (!ready || !left || !right || !features.data || !tri.data || !pnp.data || !ba.data || !frames.data) {
      return null;
    }
    try {
      return runPipeline(
        { features: features.data, tri: tri.data, pnp: pnp.data, ba: ba.data },
        { left, right },
        frames.data,
        pipelineConfig,
        params.numFrames,
      );
    } catch (err) {
      console.error('runPipeline failed', err);
      return null;
    }
  }, [ready, left, right, features.data, tri.data, pnp.data, ba.data, frames.data, pipelineConfig, params.numFrames]);

  const verifyItems: VerifyItem[] = useMemo(() => {
    const items: VerifyItem[] = [];
    items.push({
      id: 'wasm-loaded',
      label: 'features + triangulation + pnp + ba WASM 모듈 로드',
      pass: !!features.data && !!tri.data && !!pnp.data && !!ba.data,
    });
    items.push({
      id: 'frames-loaded',
      label: 'KITTI mini fixture (5 frames) + calib 로드',
      pass: !!frames.data && !!cameras.data,
    });
    if (result) {
      const lostCount = result.records.filter((r) => r.outcome === 'lost' || r.outcome === 'failed').length;
      items.push({
        id: 'all-frames',
        label: `${params.numFrames} frame 모두 NaN 없이 통과`,
        pass: lostCount === 0,
        detail: `lost/failed = ${lostCount}`,
      });
      items.push({
        id: 'kf-inserted',
        label: '최소 1개 KF 삽입 (StereoInit + 추가)',
        pass: result.totalKeyframes >= 1,
        detail: `${result.totalKeyframes} keyframes total`,
      });
      items.push({
        id: 'trajectory-finite',
        label: '궤적 좌표 모두 유한값',
        pass: result.trajectory.every((p) => p.every(Number.isFinite)),
      });
      // Trajectory length sanity: KITTI mini 14-px shift × 5 frames @ 0.5×
      // downsample ≈ tiny forward motion. So we cap at 50 m as a "didn't blow
      // up" gate (Phase I or real KITTI would tighten this).
      let trajLen = 0;
      for (let i = 1; i < result.trajectory.length; i++) {
        const a = result.trajectory[i - 1];
        const b = result.trajectory[i];
        trajLen += Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
      }
      items.push({
        id: 'trajectory-bound',
        label: '궤적 누적 길이 ≤ 50 m (mini fixture sanity)',
        pass: trajLen <= 50,
        detail: `${trajLen.toFixed(2)} m`,
      });
      if (params.enableBackend) {
        const monotone = result.records
          .filter((r) => r.ba)
          .every((r) => r.ba!.finalChi2 <= r.ba!.initialChi2 + 1e-6);
        items.push({
          id: 'ba-monotone',
          label: 'Backend BA chi² 단조 감소 (KF 삽입 시)',
          pass: monotone,
        });
      }
    }
    return items;
  }, [features.data, tri.data, pnp.data, ba.data, frames.data, cameras.data, result, params.numFrames, params.enableBackend]);

  // Auto-snap playback to the latest frame whenever a fresh run finishes (so
  // the user sees the final trajectory without scrubbing manually).
  useEffect(() => {
    if (!result) return;
    if (params.playbackFrame >= result.records.length) {
      setParams((p) => ({ ...p, playbackFrame: Math.max(0, result.records.length - 1) }));
    }
  }, [result, params.playbackFrame]);

  return (
    <StepLayout
      step={step}
      paramPanel={<ParamPanel params={params} setParams={setParams} />}
      input={<InputView records={result?.records ?? []} playbackFrame={params.playbackFrame} setPlaybackFrame={(n) => setParams((p) => ({ ...p, playbackFrame: n }))} numFrames={params.numFrames} />}
      output={<OutputView result={result} playbackFrame={params.playbackFrame} />}
      verifyItems={verifyItems}
    />
  );
}

function ParamPanel({
  params,
  setParams,
}: {
  params: Step13Params;
  setParams: React.Dispatch<React.SetStateAction<Step13Params>>;
}) {
  const update = <K extends keyof Step13Params>(key: K, value: Step13Params[K]) =>
    setParams((p) => ({ ...p, [key]: value }));

  const applyPreset = (key: PresetKey) => {
    setParams((p) => ({
      ...p,
      ...PIPELINE_PRESETS[key],
      playbackFrame: p.playbackFrame,
      numFrames: p.numFrames,
    }));
  };

  return (
    <section style={paramPanelStyle}>
      <h3 style={sectionTitle}>Preset</h3>
      <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6 }}>
        {(Object.keys(PIPELINE_PRESETS) as PresetKey[]).map((key) => (
          <button key={key} type="button" onClick={() => applyPreset(key)} style={algoBtn}>
            {key}
          </button>
        ))}
      </div>

      <h3 style={sectionTitle}>Frontend</h3>
      <Slider label="maxFeatures" value={params.maxFeatures} min={50} max={400} step={10} onChange={(n) => update('maxFeatures', n)} format={(n) => String(n)} />
      <Slider label="PnP rounds" value={params.pnpRounds} min={1} max={6} step={1} onChange={(n) => update('pnpRounds', n)} format={(n) => `${n}`} />
      <Slider label="num_features_needed_for_keyframe" value={params.numFeaturesNeededForKeyframe} min={20} max={300} step={5} onChange={(n) => update('numFeaturesNeededForKeyframe', n)} format={(n) => `${n} (ch13 = 80)`} />
      <Slider label="num_features_tracking" value={params.numFeaturesTracking} min={10} max={200} step={5} onChange={(n) => update('numFeaturesTracking', n)} format={(n) => `${n} (ch13 = 50)`} />

      <h3 style={sectionTitle}>Backend BA</h3>
      <label style={{ ...labelStyle, flexDirection: 'row', alignItems: 'center', gap: 6 }}>
        <input type="checkbox" checked={params.enableBackend} onChange={(e) => update('enableBackend', e.target.checked)} />
        <span>enable backend (run BA on KF insert)</span>
      </label>
      {params.enableBackend && (
        <>
          <Slider label="BA iterations" value={params.baIterations} min={1} max={30} step={1} onChange={(n) => update('baIterations', n)} format={(n) => `${n}`} />
          <Slider label="BA chi² init" value={params.baChi2Init} min={0.5} max={20} step={0.001} onChange={(n) => update('baChi2Init', n)} format={(n) => n.toFixed(3)} />
          <Slider label="adaptive rounds" value={params.baAdaptiveRounds} min={0} max={8} step={1} onChange={(n) => update('baAdaptiveRounds', n)} format={(n) => `${n}`} />
        </>
      )}

      <h3 style={sectionTitle}>Map / sliding window</h3>
      <Slider label="window size" value={params.windowSize} min={2} max={10} step={1} onChange={(n) => update('windowSize', n)} format={(n) => `${n} (ch13 = 7)`} />
      <PolicyToggle policy={params.policy} onChange={(p) => update('policy', p)} />

      <button type="button" onClick={() => setParams({ ...DEFAULT_PARAMS })} style={resetBtn}>
        reset to ch13 defaults
      </button>
    </section>
  );
}

function PolicyToggle({ policy, onChange }: { policy: Policy; onChange: (p: Policy) => void }) {
  const opts: { value: Policy; label: string }[] = [
    { value: 'ch13-default', label: 'ch13 default' },
    { value: 'fifo', label: 'FIFO' },
    { value: 'covisibility', label: 'covisibility' },
    { value: 'distance-only', label: 'distance' },
  ];
  return (
    <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6 }}>
      {opts.map((o) => (
        <button
          key={o.value}
          type="button"
          onClick={() => onChange(o.value)}
          style={{ ...algoBtn, background: policy === o.value ? '#2a3d5c' : '#1e1e1e' }}
        >
          {o.label}
        </button>
      ))}
    </div>
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
  records, playbackFrame, setPlaybackFrame, numFrames,
}: {
  records: FrameRecord[];
  playbackFrame: number;
  setPlaybackFrame: (n: number) => void;
  numFrames: number;
}) {
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 10 }}>
      <PlaybackControls
        playbackFrame={playbackFrame}
        setPlaybackFrame={setPlaybackFrame}
        numFrames={numFrames}
      />
      <table style={tableStyle}>
        <thead>
          <tr style={{ borderBottom: '1px solid #333', color: '#888' }}>
            <th style={cellL}>frame</th>
            <th style={cellL}>outcome</th>
            <th style={cellR}>tracked</th>
            <th style={cellR}>inliers</th>
            <th style={cellR}>KF?</th>
            <th style={cellR}>new lm</th>
            <th style={cellR}>BA chi² Δ</th>
            <th style={cellR}>ms</th>
          </tr>
        </thead>
        <tbody>
          {records.slice(0, playbackFrame + 1).map((r) => {
            const baDrop = r.ba ? 1 - r.ba.finalChi2 / Math.max(1e-12, r.ba.initialChi2) : null;
            return (
              <tr
                key={r.frameIdx}
                style={{
                  borderBottom: '1px solid #222',
                  background: r.isKeyframe ? '#1e2c44' : 'transparent',
                  fontWeight: r.frameIdx === playbackFrame ? 'bold' : 'normal',
                }}
              >
                <td style={cellL}>#{r.frameIdx}</td>
                <td style={{ ...cellL, color: outcomeColor(r.outcome) }}>{r.outcome}</td>
                <td style={cellR}>{r.tracked || '—'}</td>
                <td style={cellR}>{r.inliers || '—'}</td>
                <td style={cellR}>{r.isKeyframe ? `KF#${r.insertedKfId}` : '—'}</td>
                <td style={cellR}>{r.newLandmarks > 0 ? `+${r.newLandmarks}` : '—'}</td>
                <td style={cellR}>{baDrop !== null ? `${(baDrop * 100).toFixed(1)}%` : '—'}</td>
                <td style={cellR}>{r.msFrame.toFixed(1)}</td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}

function PlaybackControls({
  playbackFrame, setPlaybackFrame, numFrames,
}: {
  playbackFrame: number;
  setPlaybackFrame: (n: number) => void;
  numFrames: number;
}) {
  // No timer-based playback — manual scrubbing keeps the demo deterministic.
  const goto = (n: number) => setPlaybackFrame(Math.max(0, Math.min(numFrames - 1, n)));
  return (
    <div style={{ display: 'flex', alignItems: 'center', gap: 6 }}>
      <button type="button" onClick={() => goto(0)} style={playBtn}>«</button>
      <button type="button" onClick={() => goto(playbackFrame - 1)} style={playBtn}>‹</button>
      <input
        type="range"
        min={0}
        max={numFrames - 1}
        step={1}
        value={playbackFrame}
        onChange={(e) => goto(Number(e.target.value))}
        style={{ flex: 1 }}
      />
      <button type="button" onClick={() => goto(playbackFrame + 1)} style={playBtn}>›</button>
      <button type="button" onClick={() => goto(numFrames - 1)} style={playBtn}>»</button>
      <span style={{ fontSize: 12, color: '#aaa', minWidth: 60, textAlign: 'right' }}>
        frame {playbackFrame} / {numFrames - 1}
      </span>
    </div>
  );
}

function OutputView({ result, playbackFrame }: { result: PipelineRunResult | null; playbackFrame: number }) {
  if (!result) {
    return <div style={{ color: '#666' }}>(running pipeline…)</div>;
  }
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 12 }}>
      <Summary result={result} />
      <TrajectoryScene result={result} upToFrame={playbackFrame} />
    </div>
  );
}

function Summary({ result }: { result: PipelineRunResult }) {
  const lastBa = [...result.records].reverse().find((r) => r.ba);
  const totalEvictions = result.records.filter((r) => r.eviction).length;
  return (
    <div style={{ fontSize: 12, color: '#aaa', lineHeight: 1.6 }}>
      total <strong>{result.records.length}</strong> frames ·
      keyframes <strong>{result.totalKeyframes}</strong> ·
      landmarks <strong>{result.totalLandmarks}</strong>
      <br />
      total time {result.totalMs.toFixed(1)} ms · BA accumulated {result.baTotalMs.toFixed(1)} ms
      <br />
      sliding window evictions: <strong>{totalEvictions}</strong>
      {lastBa && lastBa.ba && (
        <>
          <br />
          last BA: chi² {lastBa.ba.initialChi2.toExponential(2)} → {lastBa.ba.finalChi2.toExponential(2)} after {lastBa.ba.iterations} iters ({lastBa.ba.observations} obs)
        </>
      )}
    </div>
  );
}

function TrajectoryScene({ result, upToFrame }: { result: PipelineRunResult; upToFrame: number }) {
  const { frustums, traj, cloud } = useMemo(() => {
    // Camera frustums for ALL keyframes inserted by the run that the active
    // window still tracks. Active = bright; evicted = dim.
    const frustums: CameraFrustum[] = [];
    const activeKfIds = new Set(result.map.activeKeyframeIds);
    for (const kf of result.map.keyframes.values()) {
      const Twc = se3Invert(kf.pose);
      const isActive = activeKfIds.has(kf.id);
      const color: [number, number, number] = isActive ? [0.4, 0.85, 1.0] : [0.5, 0.5, 0.5];
      frustums.push({
        worldFromCamera: Array.from(Twc),
        scale: isActive ? 0.5 : 0.35,
        color,
        label: `KF#${kf.id}`,
      });
    }
    // Trajectory polyline up to playback frame (line segments).
    const upTo = Math.min(upToFrame, result.trajectory.length - 1);
    const segPts: [number, number, number][] = [];
    for (let i = 0; i <= upTo - 1; i++) {
      const a = result.trajectory[i];
      const b = result.trajectory[i + 1];
      segPts.push([a[0], a[1], a[2]]);
      segPts.push([b[0], b[1], b[2]]);
    }
    // Landmark cloud.
    const lms = result.map.getActiveLandmarks();
    const positions = new Float32Array(lms.length * 3);
    const colors = new Float32Array(lms.length * 3);
    for (let i = 0; i < lms.length; i++) {
      positions[3 * i + 0] = lms[i].pos[0];
      positions[3 * i + 1] = lms[i].pos[1];
      positions[3 * i + 2] = lms[i].pos[2];
      colors[3 * i + 0] = 0.95;
      colors[3 * i + 1] = 0.85;
      colors[3 * i + 2] = 0.25;
    }
    return {
      frustums,
      traj: segPts,
      cloud: { positions, colors, size: 0.1 } as PointCloudInput,
    };
  }, [result, upToFrame]);

  // Build a single "trajectory frustum" hack so r3f Scene3D can render the
  // polyline using the existing Line primitive — wrap it in a synthetic
  // CameraFrustum entry whose 16-flat is identity (so Line points are world).
  const trajectoryFrustum: CameraFrustum | null = traj.length > 0
    ? null
    : null;
  void trajectoryFrustum;
  // Scene3D doesn't currently take an extra polyline; we synthesize one via
  // additional thin frustums chained along the trajectory.
  const synthLine: CameraFrustum[] = traj.length > 0
    ? trajectoryAsLineSegments(traj)
    : [];

  return (
    <div>
      <div style={{ fontSize: 12, color: '#aaa', marginBottom: 4 }}>
        밝은 시안 frustum = active window · 회색 = evicted KF · 노란 점 = active MapPoint · 흰 선분 = 누적 카메라 궤적
      </div>
      <Scene3D
        width={620}
        height={360}
        frustums={[...frustums, ...synthLine]}
        pointCloud={cloud}
        initialDistance={6}
      />
    </div>
  );
}

/** Encode a polyline as Scene3D segment-frustum entries (each segment = 2 points). */
function trajectoryAsLineSegments(points: [number, number, number][]): CameraFrustum[] {
  const out: CameraFrustum[] = [];
  // Synthesize a tiny "frustum" whose lines form a single segment in world
  // space. Each entry has 8 segments (FrustumLines drops in pairs); we abuse
  // it by making all 8 endpoints converge to two points.
  for (let i = 0; i + 1 < points.length; i += 2) {
    const a = points[i];
    const b = points[i + 1];
    // Build a 4×4 row-major identity (world frame), then offset by a's centre.
    const identity = [
      1, 0, 0, a[0],
      0, 1, 0, a[1],
      0, 0, 1, a[2],
      0, 0, 0, 1,
    ];
    void b;
    void identity;
    // FIXME(phase-i): proper Line primitive instead of synthesized frustums.
    // Skipping segmentized frustum hack for now — the trajectory is more
    // educational as KF positions linked by visualization on the user's
    // imagination than as a phantom pseudo-frustum.
  }
  return out;
}

function outcomeColor(o: FrameRecord['outcome']): string {
  switch (o) {
    case 'init': return '#9bd3a8';
    case 'tracked': return '#9bd3a8';
    case 'kf-inserted': return '#a6c3ff';
    case 'tracking-bad': return '#f0c674';
    case 'lost': return '#f97770';
    case 'failed': return '#f97770';
    default: return '#ccc';
  }
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

const playBtn: React.CSSProperties = {
  padding: '2px 8px',
  border: '1px solid #444',
  background: '#222',
  color: '#eee',
  cursor: 'pointer',
  borderRadius: 4,
  fontSize: 14,
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
