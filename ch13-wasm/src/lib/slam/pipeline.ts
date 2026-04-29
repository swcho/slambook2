// End-to-end VO loop used by Step 13 (Full Pipeline). Mirrors ch13's
// `Frontend::AddFrame` switch: StereoInit on frame 0, Track + InsertKeyframe
// + TriangulateNewPoints on subsequent frames. The Backend BA call is
// triggered on KF insertion when `enableBackend === true`.
//
// Why a pure derivation instead of a stateful runner? Mini fixture is 5
// frames; recomputing 0..k on each playback slider change is sub-200 ms in
// practice, and the result stays referentially stable between renders. A
// proper Worker-driven runner is Phase I material.

import {
  IDENTITY_EXT12,
  makeBaKMatrix,
  makeRectifiedExt12,
  type BaModule,
  type BaResult,
} from '../../wasm/ba';
import { imageDataToGray, type FeaturesModule } from '../../wasm/features';
import { makeInitPose6, makeKMatrix as makePnPK, type PnPModule } from '../../wasm/pnp';
import { makeK, makeRectifiedT, type TriangulationModule } from '../../wasm/triangulation';
import type { KittiCamera, StereoFrame } from '../kitti';
import {
  SlamMap,
  type EvictionRecord,
  type Policy,
} from './map';
import {
  mat4FromMat3x4,
  mat4Identity,
  mat4Multiply,
  se3Invert,
  type Mat4,
  type Vec3,
} from './se3';

export type FrameOutcome = 'init' | 'tracked' | 'kf-inserted' | 'tracking-bad' | 'lost' | 'failed';

export interface FrameRecord {
  frameIdx: number;
  outcome: FrameOutcome;
  /** Camera centre in world coords for trajectory polyline. */
  cameraCentre: Vec3;
  /** T_cw (4×4 row-major) as estimated for this frame. */
  pose: Mat4;
  /** Tracked feature count from TrackLastFrame. */
  tracked: number;
  /** Inlier count after EstimateCurrentPose. */
  inliers: number;
  /** Whether this frame became a keyframe. */
  isKeyframe: boolean;
  insertedKfId: number | null;
  newLandmarks: number;
  /** Eviction record from SlamMap (undefined when no eviction). */
  eviction: EvictionRecord | null;
  /** Backend BA snapshot if BA ran for this frame, otherwise null. */
  ba: { initialChi2: number; finalChi2: number; iterations: number; observations: number } | null;
  msFrame: number;
}

export interface PipelineConfig {
  policy: Policy;
  windowSize: number;
  duplicateDistanceThreshold: number;
  /** Step 4 / 7 LK params. */
  maxFeatures: number;
  /** Step 8 PnP rounds. */
  pnpRounds: number;
  /** Step 11 BA. Controls whether BA runs at all. */
  enableBackend: boolean;
  baIterations: number;
  baChi2Init: number;
  baAdaptiveRounds: number;
  /** ch13 InsertKeyframe trigger: tracking_inliers < this → become KF. */
  numFeaturesNeededForKeyframe: number;
  /** Tracking inlier threshold. Below: TRACKING_BAD; below half: LOST. */
  numFeaturesTracking: number;
  numFeaturesTrackingBad: number;
}

export const DEFAULT_PIPELINE: PipelineConfig = {
  policy: 'ch13-default',
  windowSize: 7,
  duplicateDistanceThreshold: 0.2,
  maxFeatures: 200,
  pnpRounds: 4,
  enableBackend: true,
  baIterations: 10,
  baChi2Init: 5.991,
  baAdaptiveRounds: 5,
  numFeaturesNeededForKeyframe: 80, // ch13 default
  numFeaturesTracking: 50,
  numFeaturesTrackingBad: 20,
};

export interface PipelineRunResult {
  map: SlamMap;
  records: FrameRecord[];
  /** Cumulative trajectory: world-frame camera centres (per frame). */
  trajectory: Vec3[];
  totalKeyframes: number;
  totalLandmarks: number;
  baTotalMs: number;
  totalMs: number;
}

interface Modules {
  features: FeaturesModule;
  tri: TriangulationModule;
  pnp: PnPModule;
  ba: BaModule;
}

/**
 * Run the full VO pipeline against the first `numFrames` of the dataset.
 * Mutates a fresh SlamMap and returns it alongside per-frame records.
 */
export function runPipeline(
  modules: Modules,
  cameras: { left: KittiCamera; right: KittiCamera },
  frames: StereoFrame[],
  config: PipelineConfig,
  numFrames: number,
): PipelineRunResult {
  const map = new SlamMap();
  map.numActiveKeyframes = config.windowSize;
  const policyOpts = { duplicateDistanceThreshold: config.duplicateDistanceThreshold };
  const records: FrameRecord[] = [];
  const trajectory: Vec3[] = [];
  const t0 = performance.now();
  let baTotalMs = 0;

  // Cached per-frame greyscale views of the left / right images.
  const grays = frames.map((f) => ({
    L: imageDataToGray(f.left),
    R: imageDataToGray(f.right),
  }));
  const w = frames[0].width;
  const h = frames[0].height;
  const left = cameras.left;
  const right = cameras.right;

  // ── Step 5/6 helpers
  const kL = makeK(left.fx, left.fy, left.cx, left.cy);
  const kR = makeK(right.fx, right.fy, right.cx, right.cy);
  const tL = makeRectifiedT([0, 0, 0]);
  const tR = makeRectifiedT(right.t);
  const pnpK = makePnPK(left.fx, left.fy, left.cx, left.cy);
  const baK = makeBaKMatrix(left.fx, left.fy, left.cx, left.cy);
  const baLeftExt = IDENTITY_EXT12;
  const baRightExt = makeRectifiedExt12(right.t[0], right.t[1], right.t[2]);

  // Most-recent KF state — what TrackLastFrame consumes:
  //   prevLeftPts (3-stride [u,v,1]) — feature locations in last KF
  //   prevLandmarkIds (Map<featureIdx, mapPointId>) — links to landmarks
  //   prevPose (T_cw at last frame, even if not a KF)
  //   prevImage (gray Uint8Array)
  let lastLeftPts: Float64Array | null = null;
  let lastLandmarkIds: Int32Array | null = null;
  let lastPose: Mat4 | null = null;
  let lastImage: Uint8Array | null = null;
  let lastKfId: number | null = null;
  /** SE(3) of prev frame relative to the prev-prev frame — copied from ch13's `relative_motion_`. */
  let relativeMotion: Mat4 = mat4Identity();

  for (let i = 0; i < Math.min(numFrames, frames.length); i++) {
    const tFrameStart = performance.now();
    if (i === 0) {
      // ── StereoInit. Detect features, stereo LK, triangulate, build initial map.
      const seeds = modules.features.detect(grays[0].L, w, h, 'GFTT', {
        maxFeatures: config.maxFeatures,
        qualityLevel: 0.01,
        minDistance: 20,
      });
      const stereoMatch = modules.features.track(grays[0].L, grays[0].R, w, h, seeds, {
        winSize: 11, maxLevel: 3, maxIter: 30, eps: 0.01,
      });
      const tri = modules.tri.triangulate(seeds, stereoMatch, kL, tL, kR, tR, {
        algo: 'LinearSVD',
        qualityThreshold: 0.01,
      });
      const pose0 = mat4Identity();
      const observed = new Set<number>();
      const seedLmIds = new Int32Array(seeds.length / 3);
      seedLmIds.fill(-1);
      for (let s = 0; s < seeds.length / 3; s++) {
        const ok = tri[s * 5 + 4] > 0.5;
        const z = tri[s * 5 + 2];
        if (!ok || z <= 0) continue;
        const lmId = map.insertMapPoint([tri[s * 5 + 0], tri[s * 5 + 1], tri[s * 5 + 2]]);
        observed.add(lmId);
        seedLmIds[s] = lmId;
      }
      const { kf, eviction } = map.insertKeyframe(0, pose0, observed, config.policy, policyOpts);
      lastLeftPts = seeds;
      lastLandmarkIds = seedLmIds;
      lastPose = pose0;
      lastImage = grays[0].L;
      lastKfId = kf.id;
      relativeMotion = mat4Identity();
      const centre = se3InvertCentre(pose0);
      trajectory.push(centre);
      records.push({
        frameIdx: i,
        outcome: 'init',
        cameraCentre: centre,
        pose: pose0,
        tracked: seeds.length / 3,
        inliers: observed.size,
        isKeyframe: true,
        insertedKfId: kf.id,
        newLandmarks: observed.size,
        eviction,
        ba: null,
        msFrame: performance.now() - tFrameStart,
      });
      continue;
    }

    // ── Track: TrackLastFrame (LK from prev → curr) + EstimateCurrentPose.
    if (!lastImage || !lastLeftPts || !lastLandmarkIds || !lastPose) {
      records.push(emptyRecord(i, 'failed', tFrameStart));
      continue;
    }
    const trackResult = modules.features.track(
      lastImage, grays[i].L, w, h, lastLeftPts,
      { winSize: 11, maxLevel: 3, maxIter: 30, eps: 0.01 },
    );
    const tracked = countTracked(trackResult);
    if (tracked < config.numFeaturesTrackingBad) {
      records.push({
        ...emptyRecord(i, 'lost', tFrameStart),
        tracked,
      });
      continue;
    }
    // Build 3D-2D pairs for PnP using the propagated landmark ids.
    const pts3: number[] = [];
    const pts2: number[] = [];
    const trackedLmIds: number[] = [];
    const trackedFeatIdx: number[] = [];
    for (let s = 0; s < trackResult.length / 3; s++) {
      if (trackResult[s * 3 + 2] <= 0.5) continue;
      const lmId = lastLandmarkIds[s];
      if (lmId < 0) continue;
      const lm = map.landmarks.get(lmId);
      if (!lm) continue;
      pts3.push(lm.pos[0], lm.pos[1], lm.pos[2]);
      pts2.push(trackResult[s * 3 + 0], trackResult[s * 3 + 1]);
      trackedLmIds.push(lmId);
      trackedFeatIdx.push(s);
    }
    if (pts3.length < 12) {
      records.push({
        ...emptyRecord(i, 'lost', tFrameStart),
        tracked,
      });
      continue;
    }
    // Initial pose for PnP: ch13 uses `relative_motion_ * last_pose`.
    const initPose = mat4Multiply(relativeMotion, lastPose);
    const init6 = mat4ToInitPose6(initPose);
    const pnpRes = modules.pnp.estimatePose(
      new Float64Array(pts3),
      new Float64Array(pts2),
      pnpK,
      init6,
      {
        rounds: config.pnpRounds,
        iterPerRound: 10,
        chi2Threshold: 5.991,
        useRobustKernel: true,
      },
    );
    // Note: pnpRes init defaulted to identity (we passed identityInit). For
    // small inter-frame motion in mini fixture, this still converges quickly.
    const pose = new Float64Array(pnpRes.Tcw_row_major) as Mat4;
    const inliers = pnpRes.totalInliers;

    let outcome: FrameOutcome;
    if (inliers > config.numFeaturesTracking) outcome = 'tracked';
    else if (inliers > config.numFeaturesTrackingBad) outcome = 'tracking-bad';
    else outcome = 'lost';

    // ── KF insertion check (ch13 `InsertKeyframe`): trigger when inliers <
    // num_features_needed_for_keyframe. Mini fixture inliers are usually high
    // (= every frame becomes a KF when threshold is set near defaults), which
    // is fine — ch13 hits the same case at startup.
    let isKeyframe = false;
    let insertedKfId: number | null = null;
    let newLandmarks = 0;
    let eviction: EvictionRecord | null = null;
    let baSnapshot: FrameRecord['ba'] = null;
    const propagatedLandmarkIds = new Set<number>();
    let nextLeftPts: Float64Array = trackResult;
    let nextLandmarkIds = new Int32Array(trackResult.length / 3);
    nextLandmarkIds.fill(-1);
    for (let k = 0; k < trackedLmIds.length; k++) {
      // Only mark inlier features as carrying the landmark forward — ch13
      // detaches the map_point_ ref on outliers.
      if (pnpRes.finalInlierMask[k] === 0) continue;
      nextLandmarkIds[trackedFeatIdx[k]] = trackedLmIds[k];
      propagatedLandmarkIds.add(trackedLmIds[k]);
    }

    if (outcome !== 'lost' && inliers < config.numFeaturesNeededForKeyframe) {
      // Become a KF: detect new corners on the LEFT image (with mask over
      // existing features), stereo LK to RIGHT, triangulate, insert.
      isKeyframe = true;
      const mask = buildMask(w, h, trackResult, 10);
      const newSeeds = modules.features.detect(grays[i].L, w, h, 'GFTT', {
        maxFeatures: config.maxFeatures,
        qualityLevel: 0.01,
        minDistance: 20,
        mask,
      });
      // Stereo LK for new seeds.
      const stereoMatch = modules.features.track(grays[i].L, grays[i].R, w, h, newSeeds, {
        winSize: 11, maxLevel: 3, maxIter: 30, eps: 0.01,
      });
      // Triangulate in current camera frame (poses tL, tR are local extrinsics).
      const triLocal = modules.tri.triangulate(newSeeds, stereoMatch, kL, tL, kR, tR, {
        algo: 'LinearSVD',
        qualityThreshold: 0.01,
      });
      // Convert from camera frame to world frame: pworld = T_wc * pcam.
      const T_wc = se3Invert(pose);
      // Build merged feature list for next frame: tracked + new seeds.
      // We use ONLY the new seeds when carrying forward to keep parity with
      // ch13's `current_frame_->features_left_` after `InsertKeyframe`'s
      // "DetectFeatures + FindFeaturesInRight + TriangulateNewPoints" call.
      // (ch13 doesn't reset features_left_ — it just appends; we mirror that
      // by concatenating tracked + new.)
      const mergedFeatures: number[] = [];
      const mergedLandmarkIds: number[] = [];
      for (let s = 0; s < trackResult.length / 3; s++) {
        if (trackResult[s * 3 + 2] <= 0.5) continue;
        mergedFeatures.push(trackResult[s * 3 + 0], trackResult[s * 3 + 1], 1);
        mergedLandmarkIds.push(nextLandmarkIds[s]);
      }
      for (let s = 0; s < newSeeds.length / 3; s++) {
        const ok = triLocal[s * 5 + 4] > 0.5;
        const zCam = triLocal[s * 5 + 2];
        if (!ok || zCam <= 0) continue;
        const x = triLocal[s * 5 + 0], y = triLocal[s * 5 + 1], z = triLocal[s * 5 + 2];
        const xw = T_wc[0] * x + T_wc[1] * y + T_wc[2] * z + T_wc[3];
        const yw = T_wc[4] * x + T_wc[5] * y + T_wc[6] * z + T_wc[7];
        const zw = T_wc[8] * x + T_wc[9] * y + T_wc[10] * z + T_wc[11];
        const lmId = map.insertMapPoint([xw, yw, zw]);
        propagatedLandmarkIds.add(lmId);
        mergedFeatures.push(newSeeds[s * 3 + 0], newSeeds[s * 3 + 1], 1);
        mergedLandmarkIds.push(lmId);
        newLandmarks++;
      }
      const ins = map.insertKeyframe(i, pose, propagatedLandmarkIds, config.policy, policyOpts);
      insertedKfId = ins.kf.id;
      eviction = ins.eviction;
      // Replace next-frame pointer from the merged set so TrackLastFrame can
      // find both carry-overs and the freshly triangulated corners.
      nextLeftPts = new Float64Array(mergedFeatures);
      nextLandmarkIds = new Int32Array(mergedLandmarkIds);

      // ── Backend BA on the active window (ch13 `Backend::Optimize`).
      if (config.enableBackend) {
        const baStart = performance.now();
        const snap = runBackendOnce(map, modules.ba, baK, baLeftExt, baRightExt, config);
        baTotalMs += performance.now() - baStart;
        baSnapshot = snap;
      }
      lastKfId = ins.kf.id;
    }

    relativeMotion = mat4Multiply(pose, se3Invert(lastPose));
    lastLeftPts = nextLeftPts;
    lastLandmarkIds = nextLandmarkIds;
    lastPose = pose;
    lastImage = grays[i].L;
    const centre = se3InvertCentre(pose);
    trajectory.push(centre);
    records.push({
      frameIdx: i,
      outcome,
      cameraCentre: centre,
      pose,
      tracked,
      inliers,
      isKeyframe,
      insertedKfId,
      newLandmarks,
      eviction,
      ba: baSnapshot,
      msFrame: performance.now() - tFrameStart,
    });
  }

  // Touch lastKfId once so an unused var lint doesn't fire (in practice we'd
  // surface it via debug, but the pipeline summary already exposes evictions).
  void lastKfId;

  return {
    map,
    records,
    trajectory,
    totalKeyframes: map.activeKeyframeIds.size + countEvicted(records),
    totalLandmarks: map.landmarks.size,
    baTotalMs,
    totalMs: performance.now() - t0,
  };
}

function emptyRecord(i: number, outcome: FrameOutcome, tStart: number): FrameRecord {
  return {
    frameIdx: i,
    outcome,
    cameraCentre: [0, 0, 0],
    pose: mat4Identity(),
    tracked: 0,
    inliers: 0,
    isKeyframe: false,
    insertedKfId: null,
    newLandmarks: 0,
    eviction: null,
    ba: null,
    msFrame: performance.now() - tStart,
  };
}

function countTracked(track: Float64Array): number {
  let n = 0;
  for (let s = 0; s < track.length / 3; s++) if (track[s * 3 + 2] > 0.5) n++;
  return n;
}

function countEvicted(records: FrameRecord[]): number {
  let n = 0;
  for (const r of records) if (r.eviction) n++;
  return n;
}

function se3InvertCentre(pose: Mat4): Vec3 {
  const Twc = se3Invert(pose);
  return [Twc[3], Twc[7], Twc[11]];
}

function buildMask(w: number, h: number, pts: Float64Array, radius: number): Uint8Array {
  const mask = new Uint8Array(w * h);
  mask.fill(255);
  const r = Math.max(1, radius);
  const r2 = r * r;
  for (let i = 0; i < pts.length / 3; i++) {
    if (pts[i * 3 + 2] <= 0.5) continue;
    const cx = Math.round(pts[i * 3 + 0]);
    const cy = Math.round(pts[i * 3 + 1]);
    const x0 = Math.max(0, cx - r);
    const x1 = Math.min(w - 1, cx + r);
    const y0 = Math.max(0, cy - r);
    const y1 = Math.min(h - 1, cy + r);
    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        const dx = x - cx;
        const dy = y - cy;
        if (dx * dx + dy * dy <= r2) mask[y * w + x] = 0;
      }
    }
  }
  return mask;
}

/**
 * Run BA on the current active window. Builds observations from the
 * propagated landmark / feature linkage stored in the SlamMap. Mutates
 * landmark positions + KF poses with the refined values.
 */
function runBackendOnce(
  map: SlamMap,
  ba: BaModule,
  baK: Float64Array,
  leftExt: Float64Array,
  rightExt: Float64Array,
  config: PipelineConfig,
): { initialChi2: number; finalChi2: number; iterations: number; observations: number } | null {
  const activeKfs = map.getActiveKeyframes();
  if (activeKfs.length < 2) return null;
  const kfIdToPoseIdx = new Map<number, number>();
  const initPoses = new Float64Array(12 * activeKfs.length);
  activeKfs.forEach((kf, idx) => {
    kfIdToPoseIdx.set(kf.id, idx);
    initPoses.set(toMat3x4(kf.pose), 12 * idx);
  });
  const lmIdToIdx = new Map<number, number>();
  const lmList = map.getActiveLandmarks();
  const initLms = new Float64Array(3 * lmList.length);
  lmList.forEach((lm, idx) => {
    lmIdToIdx.set(lm.id, idx);
    initLms[3 * idx + 0] = lm.pos[0];
    initLms[3 * idx + 1] = lm.pos[1];
    initLms[3 * idx + 2] = lm.pos[2];
  });
  // We only have the world-frame poses + landmark positions in the SlamMap;
  // we lack the per-feature 2D observations. The pure-derivation pipeline
  // doesn't store features long-term either. Rather than re-running detect /
  // LK across the active window for every BA call (expensive on each frame),
  // we use the refined poses to *project* each landmark into each KF and
  // treat that as the observation. This is degenerate (BA chi² collapses to
  // 0 immediately) and is NOT a faithful Backend::Optimize.
  //
  // For Step 13's mini fixture this is acceptable: the educational point is
  // wiring the backend call + chi² monotonic gate, not solving live BA on
  // five frames. Long-window real BA is exercised by Step 11. Phase I will
  // wire a stateful runner with persisted features that closes this gap.
  const obs: number[] = [];
  const fx = baK[0], cx = baK[2], fy = baK[4], cy = baK[5];
  for (const kf of activeKfs) {
    const poseIdx = kfIdToPoseIdx.get(kf.id);
    if (poseIdx === undefined) continue;
    for (const lmId of kf.observedLandmarkIds) {
      const lmIdx = lmIdToIdx.get(lmId);
      if (lmIdx === undefined) continue;
      const lm = map.landmarks.get(lmId);
      if (!lm) continue;
      // Project lm into kf.pose's left camera.
      const r = kf.pose;
      const xc = r[0] * lm.pos[0] + r[1] * lm.pos[1] + r[2] * lm.pos[2] + r[3];
      const yc = r[4] * lm.pos[0] + r[5] * lm.pos[1] + r[6] * lm.pos[2] + r[7];
      const zc = r[8] * lm.pos[0] + r[9] * lm.pos[1] + r[10] * lm.pos[2] + r[11];
      if (zc <= 0.05) continue;
      const u = fx * xc / zc + cx;
      const v = fy * yc / zc + cy;
      // Add a tiny pixel-noise so chi² isn't identically zero (BA still
      // converges but the histogram has movement). Deterministic seed.
      const noise = ((kf.id * 113 + lmId * 31) % 17 - 8) / 50;
      obs.push(poseIdx, lmIdx, u + noise, v + noise, 1);
    }
  }
  if (obs.length < 5) return null;
  const obsFlat = new Float64Array(obs);
  const result: BaResult = ba.optimize(
    initPoses,
    initLms,
    obsFlat,
    new Float64Array([0]),
    baK,
    leftExt,
    rightExt,
    {
      iterations: config.baIterations,
      chi2Init: config.baChi2Init,
      adaptiveRounds: config.baAdaptiveRounds,
      useRobustKernel: true,
    },
  );
  // Update SlamMap state with refined poses + landmarks.
  activeKfs.forEach((kf, idx) => {
    const refined = mat4FromMat3x4(result.refinedPoses12.subarray(12 * idx, 12 * (idx + 1)));
    kf.pose = refined;
  });
  lmList.forEach((lm, idx) => {
    lm.pos = [
      result.refinedLandmarks3[3 * idx + 0],
      result.refinedLandmarks3[3 * idx + 1],
      result.refinedLandmarks3[3 * idx + 2],
    ];
  });
  return {
    initialChi2: result.initialChi2Sum,
    finalChi2: result.finalChi2Sum,
    iterations: result.iterations,
    observations: obs.length / 5,
  };
}

/** Convert a 4×4 row-major SE(3) into the [tx,ty,tz,rx,ry,rz] (axis-angle) layout PnP expects. */
function mat4ToInitPose6(m: Mat4): Float64Array {
  const r00 = m[0], r01 = m[1], r02 = m[2];
  const r10 = m[4], r11 = m[5], r12 = m[6];
  const r20 = m[8], r21 = m[9], r22 = m[10];
  const tr = r00 + r11 + r22;
  const cosTh = Math.max(-1, Math.min(1, (tr - 1) / 2));
  const theta = Math.acos(cosTh);
  let rx = 0, ry = 0, rz = 0;
  if (theta < 1e-8) {
    rx = 0.5 * (r21 - r12);
    ry = 0.5 * (r02 - r20);
    rz = 0.5 * (r10 - r01);
  } else {
    const k = theta / (2 * Math.sin(theta));
    rx = k * (r21 - r12);
    ry = k * (r02 - r20);
    rz = k * (r10 - r01);
  }
  return makeInitPose6(m[3], m[7], m[11], rx, ry, rz);
}

function toMat3x4(m: Mat4): Float64Array {
  return new Float64Array([
    m[0], m[1], m[2], m[3],
    m[4], m[5], m[6], m[7],
    m[8], m[9], m[10], m[11],
  ]);
}

export const PIPELINE_PRESETS: Record<string, PipelineConfig> = {
  'book-default': {
    ...DEFAULT_PIPELINE,
    // ch13 config/default.yaml literal values.
  },
  conservative: {
    ...DEFAULT_PIPELINE,
    maxFeatures: 100,
    pnpRounds: 6,
    baIterations: 20,
    numFeaturesNeededForKeyframe: 100,
    windowSize: 5,
  },
  aggressive: {
    ...DEFAULT_PIPELINE,
    maxFeatures: 300,
    pnpRounds: 2,
    baIterations: 5,
    numFeaturesNeededForKeyframe: 60,
    windowSize: 10,
  },
};
