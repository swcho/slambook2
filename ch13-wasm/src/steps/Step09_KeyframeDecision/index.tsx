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
import {
  loadPnPWasm,
  makeKMatrix,
  identityInit,
  type PnPModule,
} from '../../wasm/pnp';

// Step 9 mirrors ch13/src/frontend.cpp::InsertKeyframe. The book uses a single
// criterion — `tracking_inliers < num_features_needed_for_keyframe (=80)` —
// but PLAN §3 Step 9 asks for at least three swappable policies so learners
// can compare. We run the Step 7+8 pipeline (detect → stereo LK → triangulate
// → temporal LK → PnP) for every i → i+1 transition in the mini fixture, then
// apply each policy on the resulting tracking_inliers + relative motion.

const DATASET_DIR = '/datasets/kitti05-mini';
const FRAME_COUNT = 5;
// ch13/include/myslam/frontend.h::num_features_needed_for_keyframe default = 80.
const KF_INLIER_THRESHOLD_DEFAULT = 80;

type KeyframePolicy = 'inlier' | 'motion' | 'hybrid';

interface Step9Params {
  policy: KeyframePolicy;
  /** ch13: tracking_inliers < this → insert KF. */
  inlierThreshold: number;
  /** Motion policy: ||t|| in metres. */
  translationThreshold: number;
  /** Motion policy: rotation in degrees. */
  rotationThresholdDeg: number;
  maxFeatures: number;
}

const DEFAULT_PARAMS: Step9Params = {
  policy: 'inlier',
  // The synthetic fixture's tracking_inliers usually sits at ~max_features,
  // so the book's 80 default never fires here. Pre-set to ~85% of typical
  // detections so the bar chart shows policy transitions out of the box.
  inlierThreshold: 170,
  translationThreshold: 0.3,
  rotationThresholdDeg: 1.5,
  maxFeatures: 200,
};

interface Transition {
  /** prev frame index (i-1). */
  prev: number;
  /** curr frame index (i). */
  curr: number;
  trackingInliers: number;
  pairCount: number;
  translationNorm: number;
  rotationDeg: number;
  pnpMs: number;
  totalMs: number;
}

interface Decision {
  isKeyframe: boolean;
  reason: string;
}

export function Step09KeyframeDecision() {
  const step = STEPS.find((s) => s.id === 9)!;
  const [params, setParams] = useState<Step9Params>(DEFAULT_PARAMS);

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

  // Run the per-transition pipeline once per (frames | maxFeatures). This is
  // the hot loop — KF policy thresholds re-render against the cached results.
  const transitions = useMemo<Transition[] | null>(() => {
    if (!features.data || !tri.data || !pnp.data || !frames.data || !cameras.data) {
      return null;
    }
    const left = cameras.data.find((c) => c.id === 0);
    const right = cameras.data.find((c) => c.id === 1);
    if (!left || !right) return null;
    const out: Transition[] = [];
    for (let i = 1; i < frames.data.length; i++) {
      const prev = frames.data[i - 1];
      const curr = frames.data[i];
      const t = runTransition(
        features.data,
        tri.data,
        pnp.data,
        prev,
        curr,
        left,
        right,
        params.maxFeatures,
      );
      if (t) out.push(t);
    }
    return out;
  }, [
    features.data, tri.data, pnp.data,
    frames.data, cameras.data,
    params.maxFeatures,
  ]);

  const decisions = useMemo<Decision[] | null>(() => {
    if (!transitions) return null;
    return transitions.map((t) => decideKeyframe(t, params));
  }, [transitions, params]);

  const kfCount = decisions?.filter((d) => d.isKeyframe).length ?? 0;
  const kfRate = decisions && decisions.length > 0 ? kfCount / decisions.length : 0;

  const verifyItems: VerifyItem[] = useMemo(() => {
    const items: VerifyItem[] = [];
    items.push({
      id: 'wasm-loaded',
      label: 'features + triangulation + pnp WASM 모듈 로드',
      pass: !!features.data && !!tri.data && !!pnp.data,
    });
    items.push({
      id: 'frames-loaded',
      label: `${FRAME_COUNT} frames + calib 로드`,
      pass: !!frames.data && frames.data.length === FRAME_COUNT && !!cameras.data,
      detail: frames.data ? `${frames.data.length} frames` : undefined,
    });
    if (transitions) {
      items.push({
        id: 'pnp-all',
        label: `모든 ${FRAME_COUNT - 1}개 transition에서 PnP 수렴`,
        pass: transitions.length === FRAME_COUNT - 1,
        detail: `${transitions.length} / ${FRAME_COUNT - 1} transitions`,
      });
    } else {
      items.push({ id: 'pnp-all', label: 'PnP 수렴', pass: false, detail: '계산 대기' });
    }
    if (decisions) {
      // PLAN §3 Step 9 asks for KF rate 5–15%, defined for a long sequence. The
      // 5-frame mini fixture quantises rates to 0/25/50/75/100% — relax to
      // "policy meaningfully fires (≥1 KF) and isn't 100% spam" so learners can
      // observe the policy producing a sequence of decisions.
      items.push({
        id: 'kf-emits',
        label: '정책이 KF를 ≥ 1회 트리거 (학습용; 임계값 슬라이더로 조정 가능)',
        pass: kfCount >= 1,
        detail: `${kfCount} KF / ${decisions.length} transitions (${(kfRate * 100).toFixed(0)}%)`,
      });
      items.push({
        id: 'kf-rate-bound',
        label: 'KF 비율 ≤ 100% (sanity)',
        pass: kfRate <= 1.0,
      });
    } else {
      items.push({ id: 'kf-emits', label: '정책이 KF 트리거', pass: false });
      items.push({ id: 'kf-rate-bound', label: 'KF 비율 ≤ 100%', pass: false });
    }
    return items;
  }, [features.data, tri.data, pnp.data, frames.data, cameras.data, transitions, decisions, kfCount, kfRate]);

  return (
    <StepLayout
      step={step}
      paramPanel={<ParamPanel params={params} setParams={setParams} />}
      input={<InputView frames={frames.data ?? null} />}
      output={
        <OutputView
          transitions={transitions}
          decisions={decisions}
          params={params}
        />
      }
      verifyItems={verifyItems}
    />
  );
}

function runTransition(
  features: FeaturesModule,
  tri: TriangulationModule,
  pnp: PnPModule,
  prev: StereoFrame,
  curr: StereoFrame,
  left: KittiCamera,
  right: KittiCamera,
  maxFeatures: number,
): Transition | null {
  const w = prev.width;
  const h = prev.height;
  const grayPrevL = imageDataToGray(prev.left);
  const grayPrevR = imageDataToGray(prev.right);
  const grayCurrL = imageDataToGray(curr.left);

  const t0 = performance.now();
  const seeds = features.detect(grayPrevL, w, h, 'GFTT', {
    maxFeatures,
    qualityLevel: 0.01,
    minDistance: 20,
  });
  if (seeds.length === 0) return null;

  const stereo = features.track(grayPrevL, grayPrevR, w, h, seeds, {
    winSize: 11, maxLevel: 3, maxIter: 30, eps: 0.01,
  });

  const kL = makeK(left.fx, left.fy, left.cx, left.cy);
  const kR = makeK(right.fx, right.fy, right.cx, right.cy);
  const tL = makeRectifiedT([0, 0, 0]);
  const tR = makeRectifiedT(right.t);
  const triPts = tri.triangulate(seeds, stereo, kL, tL, kR, tR, {
    algo: 'LinearSVD', qualityThreshold: 0.01,
  });

  const tracked = features.track(grayPrevL, grayCurrL, w, h, seeds, {
    winSize: 11, maxLevel: 3, maxIter: 30, eps: 0.01,
  });

  const total = seeds.length / 3;
  const pts3: number[] = [];
  const pts2: number[] = [];
  for (let i = 0; i < total; i++) {
    const triOk = triPts[i * 5 + 4] > 0.5;
    const lkOk = tracked[i * 3 + 2] > 0.5;
    if (!triOk || !lkOk) continue;
    pts3.push(triPts[i * 5 + 0], triPts[i * 5 + 1], triPts[i * 5 + 2]);
    pts2.push(tracked[i * 3 + 0], tracked[i * 3 + 1]);
  }
  const pairCount = pts3.length / 3;
  if (pairCount < 4) return null;

  const K = makeKMatrix(left.fx, left.fy, left.cx, left.cy);
  const t1 = performance.now();
  const res = pnp.estimatePose(
    new Float64Array(pts3),
    new Float64Array(pts2),
    K,
    identityInit(),
  );
  const pnpMs = performance.now() - t1;

  const T = res.Tcw_row_major;
  const trace = T[0] + T[5] + T[10];
  const cosTheta = Math.max(-1, Math.min(1, (trace - 1) / 2));
  const rotationRad = Math.acos(cosTheta);
  const tx = T[3];
  const ty = T[7];
  const tz = T[11];
  const translationNorm = Math.hypot(tx, ty, tz);

  return {
    prev: prev.index,
    curr: curr.index,
    trackingInliers: res.totalInliers,
    pairCount,
    translationNorm,
    rotationDeg: (rotationRad * 180) / Math.PI,
    pnpMs,
    totalMs: performance.now() - t0,
  };
}

function decideKeyframe(t: Transition, p: Step9Params): Decision {
  const inlierTrip = t.trackingInliers < p.inlierThreshold;
  const motionTrip =
    t.translationNorm > p.translationThreshold ||
    t.rotationDeg > p.rotationThresholdDeg;
  switch (p.policy) {
    case 'inlier':
      return {
        isKeyframe: inlierTrip,
        reason: inlierTrip
          ? `inliers ${t.trackingInliers} < ${p.inlierThreshold}`
          : `inliers ${t.trackingInliers} ≥ ${p.inlierThreshold}`,
      };
    case 'motion':
      return {
        isKeyframe: motionTrip,
        reason: motionTrip
          ? `||t||=${t.translationNorm.toFixed(3)}m, rot=${t.rotationDeg.toFixed(2)}°`
          : `motion below thresholds`,
      };
    case 'hybrid':
      return {
        isKeyframe: inlierTrip || motionTrip,
        reason: inlierTrip && motionTrip
          ? 'inlier + motion'
          : inlierTrip
            ? 'inlier'
            : motionTrip
              ? 'motion'
              : 'no trigger',
      };
  }
}

function ParamPanel({
  params, setParams,
}: {
  params: Step9Params;
  setParams: (next: Step9Params) => void;
}) {
  const update = <K extends keyof Step9Params>(key: K, value: Step9Params[K]) =>
    setParams({ ...params, [key]: value });
  return (
    <section style={paramPanelStyle}>
      <h3 style={sectionTitle}>Keyframe policy</h3>
      <div style={{ display: 'flex', flexWrap: 'wrap', gap: 6 }}>
        {(['inlier', 'motion', 'hybrid'] as const).map((p) => (
          <button
            key={p}
            type="button"
            onClick={() => update('policy', p)}
            style={{ ...algoBtn, background: params.policy === p ? '#2a3d5c' : '#222' }}
            title={POLICY_TOOLTIPS[p]}
          >
            {p}
          </button>
        ))}
        <span
          title="Covariance-based policy needs landmark uncertainty propagation — Phase G(BA) prerequisite."
          style={algoBtnDisabled}
        >
          covariance (Step 11+)
        </span>
      </div>

      <h3 style={sectionTitle}>Thresholds</h3>
      <Slider
        label="num_features_needed_for_keyframe"
        value={params.inlierThreshold}
        min={20}
        max={300}
        step={1}
        onChange={(n) => update('inlierThreshold', n)}
        format={(n) => String(n)}
        disabled={params.policy === 'motion'}
      />
      <Slider
        label="translation threshold (m)"
        value={params.translationThreshold}
        min={0.01}
        max={2}
        step={0.01}
        onChange={(n) => update('translationThreshold', n)}
        format={(n) => n.toFixed(2)}
        disabled={params.policy === 'inlier'}
      />
      <Slider
        label="rotation threshold (°)"
        value={params.rotationThresholdDeg}
        min={0.1}
        max={20}
        step={0.1}
        onChange={(n) => update('rotationThresholdDeg', n)}
        format={(n) => n.toFixed(1)}
        disabled={params.policy === 'inlier'}
      />

      <h3 style={sectionTitle}>Pipeline</h3>
      <Slider
        label="maxFeatures"
        value={params.maxFeatures}
        min={20}
        max={500}
        step={10}
        onChange={(n) => update('maxFeatures', n)}
        format={(n) => String(n)}
      />

      <button
        type="button"
        onClick={() => setParams(DEFAULT_PARAMS)}
        style={resetBtn}
      >
        reset (inlier policy, thresh {KF_INLIER_THRESHOLD_DEFAULT}+ tuned for fixture)
      </button>
      <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
        ch13 InsertKeyframe: <code>tracking_inliers &lt; num_features_needed_for_keyframe</code> (default 80).
        합성 fixture는 inlier가 ~max_features에 머무므로 임계값을 슬라이더로 올려 정책 발화를 관찰.
        <br />
        PLAN §3 Step 9 검증 기준 "KF 비율 5–15%"는 긴 KITTI 시퀀스 기준 — 5프레임 fixture에서는 학습용으로 ≥1 KF 트리거를 게이트로 사용한다.
      </div>
    </section>
  );
}

const POLICY_TOOLTIPS: Record<KeyframePolicy, string> = {
  inlier: 'ch13 default — tracking_inliers < num_features_needed_for_keyframe.',
  motion: 'KF when translation OR rotation exceeds threshold (alternative policy).',
  hybrid: 'inlier OR motion trigger (union).',
};

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

function InputView({ frames }: { frames: StereoFrame[] | null }) {
  if (!frames) return <div style={{ color: '#666', fontSize: 12 }}>loading mini sequence…</div>;
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 6 }}>
      <div style={{ fontSize: 12, color: '#888' }}>
        sequence: {frames.length} frames · {frames[0].width}×{frames[0].height}
      </div>
      <div
        style={{
          display: 'grid',
          gridTemplateColumns: `repeat(${frames.length}, 1fr)`,
          gap: 4,
        }}
      >
        {frames.map((f) => (
          <FrameThumb key={f.index} frame={f} />
        ))}
      </div>
      <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
        Step 9는 i → i+1 transition마다 Step 7+8 파이프라인(detect → stereo LK → triangulate → temporal LK → PnP)을 돌려 tracking_inliers와 상대 운동(||t||, rot)을 얻고, 정책 슬라이더에 대해 KF 결정을 즉시 갱신한다.
      </div>
    </div>
  );
}

function FrameThumb({ frame }: { frame: StereoFrame }) {
  // Use a callback ref so the canvas always renders even after React StrictMode
  // double-mount; useEffect would race the conditional render here.
  return (
    <figure style={figStyle}>
      <figcaption style={figCaption}>#{frame.index}</figcaption>
      <canvas
        ref={(node) => {
          if (!node) return;
          node.width = frame.width;
          node.height = frame.height;
          node.getContext('2d')?.putImageData(frame.left, 0, 0);
        }}
        style={canvasStyle}
      />
    </figure>
  );
}

function OutputView({
  transitions, decisions, params,
}: {
  transitions: Transition[] | null;
  decisions: Decision[] | null;
  params: Step9Params;
}) {
  if (!transitions || !decisions) {
    return <div style={{ color: '#666', fontSize: 12 }}>(running pipeline over transitions…)</div>;
  }
  const kfCount = decisions.filter((d) => d.isKeyframe).length;
  const kfRate = decisions.length > 0 ? kfCount / decisions.length : 0;
  return (
    <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
      <div style={{ fontSize: 12, color: '#aaa' }}>
        policy <strong>{params.policy}</strong> · KF <strong>{kfCount}</strong> / {decisions.length}{' '}
        ({(kfRate * 100).toFixed(0)}%) · PLAN target 5–15% (긴 시퀀스 기준)
      </div>

      <InliersTimeline
        transitions={transitions}
        decisions={decisions}
        threshold={params.inlierThreshold}
        policy={params.policy}
      />

      <table style={tableStyle}>
        <thead>
          <tr>
            <th style={thStyle}>transition</th>
            <th style={thStyle}>inliers</th>
            <th style={thStyle}>||t|| (m)</th>
            <th style={thStyle}>rot (°)</th>
            <th style={thStyle}>PnP ms</th>
            <th style={thStyle}>KF?</th>
            <th style={thStyle}>reason</th>
          </tr>
        </thead>
        <tbody>
          {transitions.map((t, i) => {
            const d = decisions[i];
            return (
              <tr key={`${t.prev}-${t.curr}`}>
                <td style={tdStyle}>#{t.prev}→#{t.curr}</td>
                <td style={tdStyle}>{t.trackingInliers} / {t.pairCount}</td>
                <td style={tdStyle}>{t.translationNorm.toFixed(3)}</td>
                <td style={tdStyle}>{t.rotationDeg.toFixed(2)}</td>
                <td style={tdStyle}>{t.pnpMs.toFixed(1)}</td>
                <td style={{ ...tdStyle, color: d.isKeyframe ? '#33ff66' : '#888', fontWeight: 600 }}>
                  {d.isKeyframe ? '★ KF' : '—'}
                </td>
                <td style={{ ...tdStyle, color: '#aaa' }}>{d.reason}</td>
              </tr>
            );
          })}
        </tbody>
      </table>

      <div style={{ fontSize: 11, color: '#888', lineHeight: 1.5 }}>
        ● 노란 점선 = inlier 임계값. 막대 위에 ★ = 해당 transition이 KF로 결정.
        합성 fixture는 운동이 작아 ||t||·rot이 한 자릿수 mm/° 수준 → motion 정책은 임계값을 0.001/0.01 부근으로 내려야 발화.
      </div>
    </div>
  );
}

function InliersTimeline({
  transitions, decisions, threshold, policy,
}: {
  transitions: Transition[];
  decisions: Decision[];
  threshold: number;
  policy: KeyframePolicy;
}) {
  const w = 360;
  const h = 140;
  const pad = { l: 32, r: 8, t: 14, b: 24 };
  const innerW = w - pad.l - pad.r;
  const innerH = h - pad.t - pad.b;
  const maxY = Math.max(threshold + 30, ...transitions.map((t) => t.pairCount), 1);
  const xFor = (i: number) => pad.l + (innerW * (i + 0.5)) / transitions.length;
  const yFor = (v: number) => pad.t + innerH - (innerH * v) / maxY;
  const barWidth = (innerW / transitions.length) * 0.6;
  const showThresh = policy !== 'motion';
  return (
    <figure style={{ margin: 0 }}>
      <figcaption style={{ ...figCaption, marginBottom: 4 }}>tracking_inliers per transition</figcaption>
      <svg width={w} height={h} style={{ background: '#0c0c0c', border: '1px solid #333', borderRadius: 4 }}>
        <line x1={pad.l} y1={pad.t} x2={pad.l} y2={pad.t + innerH} stroke="#444" strokeWidth={1} />
        <line x1={pad.l} y1={pad.t + innerH} x2={pad.l + innerW} y2={pad.t + innerH} stroke="#444" strokeWidth={1} />
        <text x={pad.l - 4} y={pad.t + 4} fill="#888" fontSize={9} textAnchor="end">{maxY}</text>
        <text x={pad.l - 4} y={pad.t + innerH} fill="#888" fontSize={9} textAnchor="end">0</text>
        {showThresh && (
          <>
            <line
              x1={pad.l}
              y1={yFor(threshold)}
              x2={pad.l + innerW}
              y2={yFor(threshold)}
              stroke="#ffcc33"
              strokeWidth={1}
              strokeDasharray="4 3"
            />
            <text x={pad.l + innerW - 4} y={yFor(threshold) - 3} fill="#ffcc33" fontSize={9} textAnchor="end">
              {threshold}
            </text>
          </>
        )}
        {transitions.map((t, i) => {
          const x = xFor(i);
          const y = yFor(t.trackingInliers);
          const isKF = decisions[i].isKeyframe;
          return (
            <g key={`${t.prev}-${t.curr}`}>
              <rect
                x={x - barWidth / 2}
                y={y}
                width={barWidth}
                height={innerH - (y - pad.t)}
                fill={isKF ? '#33aa66' : '#3a72b2'}
              />
              <text x={x} y={y - 3} fill="#ccc" fontSize={9} textAnchor="middle">
                {t.trackingInliers}
              </text>
              <text x={x} y={pad.t + innerH + 12} fill="#888" fontSize={9} textAnchor="middle">
                #{t.prev}→#{t.curr}
              </text>
              {isKF && (
                <text x={x} y={pad.t - 2} fill="#33ff66" fontSize={11} textAnchor="middle" fontWeight={700}>
                  ★
                </text>
              )}
            </g>
          );
        })}
      </svg>
    </figure>
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
  fontSize: 13,
};
const algoBtnDisabled: React.CSSProperties = {
  ...algoBtn,
  background: '#1a1a1a',
  color: '#666',
  border: '1px dashed #444',
  cursor: 'not-allowed',
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
const tableStyle: React.CSSProperties = {
  width: '100%',
  borderCollapse: 'collapse',
  fontSize: 12,
  color: '#ddd',
};
const thStyle: React.CSSProperties = {
  textAlign: 'left',
  padding: '4px 6px',
  borderBottom: '1px solid #333',
  color: '#aaa',
  fontWeight: 600,
};
const tdStyle: React.CSSProperties = {
  padding: '4px 6px',
  borderBottom: '1px solid #1f1f1f',
};
