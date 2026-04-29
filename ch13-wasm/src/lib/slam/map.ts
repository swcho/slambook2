// Persistent KeyFrame / MapPoint store + sliding-window policies.
// Mirrors ch13/include/myslam/map.h + src/map.cpp::RemoveOldKeyframe + the
// observation back-references kept in mappoint.h.
//
// Used by Step 12 (sliding-window comparison) and Step 13 (full pipeline).

import {
  type Mat4,
  type Vec3,
  mat4Multiply,
  se3Invert,
  se3LogNorm,
  se3Translation,
} from './se3.ts';

export interface KeyFrame {
  id: number;
  /** Dataset frame index (origin of the imagery / poses). */
  frameId: number;
  /** Insertion order — used by FIFO and for chronological visualization. */
  insertOrder: number;
  /** T_cw row-major 4×4. */
  pose: Mat4;
  /** Indices into Map.landmarks observed by this KF (left + right cameras). */
  observedLandmarkIds: Set<number>;
}

export interface MapPoint {
  id: number;
  pos: Vec3;
  /** Set of KF ids that observe this landmark (mirrors observed_times_). */
  observingKfIds: Set<number>;
}

export type Policy = 'ch13-default' | 'fifo' | 'covisibility' | 'distance-only';

export interface PolicyOptions {
  /** ch13 default: distance < this → treat as duplicate. ch13 hard-codes 0.2. */
  duplicateDistanceThreshold: number;
}

export interface EvictionRecord {
  evictedKfId: number;
  reason: string;
  /** Key metric used by the policy for the chosen KF. */
  metric: number;
  /** Number of active landmarks dropped by `cleanMap`. */
  landmarksRemoved: number;
}

export class SlamMap {
  // Persistent stores. Use plain Map<number,...> for ordered iteration semantics.
  readonly keyframes = new Map<number, KeyFrame>();
  readonly landmarks = new Map<number, MapPoint>();
  readonly activeKeyframeIds = new Set<number>();
  readonly activeLandmarkIds = new Set<number>();

  numActiveKeyframes = 7; // ch13 Map default

  private nextKfId = 0;
  private nextMpId = 0;
  private nextInsertOrder = 0;

  /**
   * Insert a keyframe (and pre-existing landmark links). Returns the eviction
   * record if the active window was trimmed.
   */
  insertKeyframe(
    frameId: number,
    pose: Mat4,
    observedLandmarkIds: Set<number>,
    policy: Policy,
    opts: PolicyOptions,
  ): { kf: KeyFrame; eviction: EvictionRecord | null } {
    const kf: KeyFrame = {
      id: this.nextKfId++,
      frameId,
      insertOrder: this.nextInsertOrder++,
      pose,
      observedLandmarkIds,
    };
    this.keyframes.set(kf.id, kf);
    this.activeKeyframeIds.add(kf.id);
    for (const lmId of observedLandmarkIds) {
      const lm = this.landmarks.get(lmId);
      if (!lm) continue;
      lm.observingKfIds.add(kf.id);
      this.activeLandmarkIds.add(lmId);
    }
    let eviction: EvictionRecord | null = null;
    if (this.activeKeyframeIds.size > this.numActiveKeyframes) {
      eviction = this.removeOldKeyframe(kf, policy, opts);
    }
    return { kf, eviction };
  }

  /** Insert a new MapPoint and return its assigned id. */
  insertMapPoint(pos: Vec3): number {
    const id = this.nextMpId++;
    this.landmarks.set(id, { id, pos, observingKfIds: new Set() });
    this.activeLandmarkIds.add(id);
    return id;
  }

  /** Read-only snapshot of the active KF list (insertion order). */
  getActiveKeyframes(): KeyFrame[] {
    const out: KeyFrame[] = [];
    for (const id of this.activeKeyframeIds) {
      const kf = this.keyframes.get(id);
      if (kf) out.push(kf);
    }
    out.sort((a, b) => a.insertOrder - b.insertOrder);
    return out;
  }

  getActiveLandmarks(): MapPoint[] {
    const out: MapPoint[] = [];
    for (const id of this.activeLandmarkIds) {
      const lm = this.landmarks.get(id);
      if (lm) out.push(lm);
    }
    return out;
  }

  /**
   * Run the configured eviction policy. Mirrors ch13 `Map::RemoveOldKeyframe`
   * for the ch13-default policy; FIFO/covisibility/distance-only are
   * educational alternatives surfaced in the Step 12 UI.
   */
  removeOldKeyframe(
    currentKf: KeyFrame,
    policy: Policy,
    opts: PolicyOptions,
  ): EvictionRecord | null {
    const candidates = this.getActiveKeyframes().filter((kf) => kf.id !== currentKf.id);
    if (candidates.length === 0) return null;

    const Twc = se3Invert(currentKf.pose);
    const distances = new Map<number, number>();
    for (const kf of candidates) {
      // (kf.pose · Twc).log().norm() — ch13 metric.
      distances.set(kf.id, se3LogNorm(mat4Multiply(kf.pose, Twc)));
    }

    let evictedId: number;
    let reason: string;
    let metric = 0;
    switch (policy) {
      case 'ch13-default': {
        let minDis = Infinity;
        let maxDis = -Infinity;
        let minId = -1;
        let maxId = -1;
        for (const kf of candidates) {
          const d = distances.get(kf.id)!;
          if (d < minDis) { minDis = d; minId = kf.id; }
          if (d > maxDis) { maxDis = d; maxId = kf.id; }
        }
        if (minDis < opts.duplicateDistanceThreshold) {
          evictedId = minId;
          metric = minDis;
          reason = `closest (d=${minDis.toFixed(3)} < ${opts.duplicateDistanceThreshold}) — ch13 duplicate branch`;
        } else {
          evictedId = maxId;
          metric = maxDis;
          reason = `farthest (no duplicates within ${opts.duplicateDistanceThreshold} m) — ch13 diversity branch`;
        }
        break;
      }
      case 'fifo': {
        // Drop the oldest non-current KF.
        let oldestOrder = Infinity;
        evictedId = -1;
        for (const kf of candidates) {
          if (kf.insertOrder < oldestOrder) {
            oldestOrder = kf.insertOrder;
            evictedId = kf.id;
          }
        }
        metric = oldestOrder;
        reason = `oldest insertion order ${oldestOrder}`;
        break;
      }
      case 'covisibility': {
        // Drop the active KF that shares the FEWEST landmarks with the
        // current frame (lowest co-visibility → least useful for BA on the
        // current state).
        let minShared = Infinity;
        evictedId = -1;
        for (const kf of candidates) {
          let shared = 0;
          for (const lmId of kf.observedLandmarkIds) {
            if (currentKf.observedLandmarkIds.has(lmId)) shared++;
          }
          if (shared < minShared) {
            minShared = shared;
            evictedId = kf.id;
          }
        }
        metric = minShared;
        reason = `fewest shared landmarks (${minShared}) with current KF`;
        break;
      }
      case 'distance-only': {
        // PLAN §3 Step 12 "거리 기반" alt: just drop the closest, no
        // duplicate threshold. Highlights the failure mode where eviction
        // collapses spatial diversity.
        let minDis = Infinity;
        evictedId = -1;
        for (const kf of candidates) {
          const d = distances.get(kf.id)!;
          if (d < minDis) { minDis = d; evictedId = kf.id; }
        }
        metric = minDis;
        reason = `closest only (d=${minDis.toFixed(3)})`;
        break;
      }
      default:
        return null;
    }

    if (evictedId < 0) return null;
    const evictedKf = this.keyframes.get(evictedId);
    if (!evictedKf) return null;

    // Drop active flag and back-references.
    this.activeKeyframeIds.delete(evictedId);
    for (const lmId of evictedKf.observedLandmarkIds) {
      const lm = this.landmarks.get(lmId);
      if (lm) lm.observingKfIds.delete(evictedId);
    }
    const landmarksRemoved = this.cleanMap();
    return { evictedKfId: evictedId, reason, metric, landmarksRemoved };
  }

  /** Drop active landmarks no longer observed by any active KF. */
  cleanMap(): number {
    let removed = 0;
    for (const lmId of [...this.activeLandmarkIds]) {
      const lm = this.landmarks.get(lmId);
      if (!lm) continue;
      // observingKfIds tracks ALL observers; consider the landmark active iff
      // at least one ACTIVE KF still observes it.
      let stillActive = false;
      for (const kfId of lm.observingKfIds) {
        if (this.activeKeyframeIds.has(kfId)) { stillActive = true; break; }
      }
      if (!stillActive) {
        this.activeLandmarkIds.delete(lmId);
        removed++;
      }
    }
    return removed;
  }

  /** Compute the variance of camera-translation across the active window. */
  activePoseSpread(): { meanT: Vec3; variance: number; n: number } {
    const ts: Vec3[] = [];
    for (const kf of this.getActiveKeyframes()) {
      // Translation of T_cw is in the camera frame; we want the world-frame
      // camera centre = -R^T t. Inverting the pose handles that.
      const Twc = se3Invert(kf.pose);
      ts.push(se3Translation(Twc));
    }
    if (ts.length === 0) return { meanT: [0, 0, 0], variance: 0, n: 0 };
    const mean: Vec3 = [0, 0, 0];
    for (const t of ts) {
      mean[0] += t[0]; mean[1] += t[1]; mean[2] += t[2];
    }
    mean[0] /= ts.length; mean[1] /= ts.length; mean[2] /= ts.length;
    let variance = 0;
    for (const t of ts) {
      const dx = t[0] - mean[0];
      const dy = t[1] - mean[1];
      const dz = t[2] - mean[2];
      variance += dx * dx + dy * dy + dz * dz;
    }
    variance /= ts.length;
    return { meanT: mean, variance, n: ts.length };
  }
}

/**
 * Synthetic KF stream generator for Step 12. Produces a 10-KF chain that
 * exercises every policy branch:
 *   - 5 forward-moving KFs (translation only, +z)
 *   - 2 duplicates clustered near KF #3 (ch13 closest branch)
 *   - 3 more forward-moving KFs at increasing distances
 *
 * Returns absolute T_cw poses + indices into a synthetic landmark set so we
 * can also exercise the covisibility policy.
 */
export interface SyntheticKfStream {
  poses: Mat4[];
  /** For each KF: which landmark ids it observes. */
  observedLandmarkIds: Set<number>[];
  /** Total number of synthetic landmarks generated. */
  numLandmarks: number;
  /** Synthetic landmark positions (Vec3[] in world frame). */
  landmarkPositions: Vec3[];
}

export function buildSyntheticKfStream(
  numForwardA = 5,
  numDuplicates = 2,
  numForwardB = 3,
): SyntheticKfStream {
  const poses: Mat4[] = [];
  const observedLandmarkIds: Set<number>[] = [];
  const landmarkPositions: Vec3[] = [];
  // Generate landmarks first so all KFs can refer to their ids consistently.
  // Layout: 80 points in a rough 4×4×5 grid in front of the cameras.
  const numLandmarks = 80;
  let seed = 0xc13d;
  const rand = () => {
    seed = (seed * 1103515245 + 12345) & 0x7fffffff;
    return seed / 0x7fffffff;
  };
  for (let i = 0; i < numLandmarks; i++) {
    landmarkPositions.push([
      (rand() - 0.5) * 6,
      (rand() - 0.5) * 4,
      4 + rand() * 12,
    ]);
  }

  // 5 forward-moving KFs: +0.6 m / step in +z direction.
  for (let i = 0; i < numForwardA; i++) {
    const tz = -i * 0.6; // T_cw camera looks at world; world recedes
    poses.push(new Float64Array([
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, tz,
      0, 0, 0, 1,
    ]));
  }
  // Duplicate cluster — perturb the 4th forward KF slightly so the closest
  // branch fires.
  const baseTz = -(numForwardA - 1) * 0.6;
  for (let i = 0; i < numDuplicates; i++) {
    const dx = 0.05 * (i + 1);
    const dz = 0.05 * (i + 1);
    poses.push(new Float64Array([
      1, 0, 0, dx,
      0, 1, 0, 0,
      0, 0, 1, baseTz + dz,
      0, 0, 0, 1,
    ]));
  }
  // 3 more forward-moving KFs, picking up from the duplicate-end position.
  for (let i = 1; i <= numForwardB; i++) {
    const tz = baseTz - i * 1.0;
    poses.push(new Float64Array([
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, tz,
      0, 0, 0, 1,
    ]));
  }

  // Observation assignment: simulate landmark visibility based on a
  // viewing-frustum heuristic. Each KF observes ~20 landmarks closest to its
  // optical axis; nearby KFs share many of those landmarks (good for the
  // covisibility policy demo).
  for (const pose of poses) {
    const Twc = se3Invert(pose);
    const cx = Twc[3], cz = Twc[11];
    const visible = new Set<number>();
    const dists: { id: number; d: number }[] = [];
    for (let i = 0; i < numLandmarks; i++) {
      const lp = landmarkPositions[i];
      // Reject landmarks behind the camera (forward axis = world +z but
      // T_cw flips it). For simple +z forward stream above, check that
      // landmark z > camera z.
      const camToLm = lp[2] - cz;
      if (camToLm < 1.0 || camToLm > 20) continue;
      const dx = lp[0] - cx;
      const dy = lp[1];
      const d = Math.hypot(dx, dy);
      dists.push({ id: i, d });
    }
    dists.sort((a, b) => a.d - b.d);
    for (let i = 0; i < Math.min(20, dists.length); i++) {
      visible.add(dists[i].id);
    }
    observedLandmarkIds.push(visible);
  }
  return { poses, observedLandmarkIds, numLandmarks, landmarkPositions };
}
