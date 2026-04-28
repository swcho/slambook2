// Loader for the Phase G / Step 11 Bundle Adjustment WASM module.
// Mirrors the Embind surface declared in
// wasm-src/myslam/bindings/bind_ba.cpp.

export interface BaOptions {
  /** LM iterations (single optimize call, then adaptive chi² loop). ch13 default = 10. */
  iterations?: number;
  /** Initial chi² threshold. ch13 default = 5.991 (95% χ²₂). */
  chi2Init?: number;
  /** Maximum number of times the adaptive loop will double the threshold. ch13 default = 5. */
  adaptiveRounds?: number;
  /** Adaptive loop terminates once inlier_ratio > this value. ch13 default = 0.5. */
  adaptiveInlierRatio?: number;
  /** Disable the Huber kernel entirely. ch13 default = true (kernel on). */
  useRobustKernel?: boolean;
  /** Huber δ. ch13 default = chi2Init (book uses literal 5.991, not √5.991). */
  huberDelta?: number;
}

export interface BaResult {
  /** length 12*P — row-major 3×4 per pose [R|t] (T_cw). */
  refinedPoses12: Float64Array;
  /** length 3*L — world-frame XYZ per landmark. */
  refinedLandmarks3: Float64Array;
  /** length O — per-edge chi² before optimization. */
  perEdgeChi2Initial: Float64Array;
  /** length O — per-edge chi² after the full adaptive loop. */
  perEdgeChi2Final: Float64Array;
  /** length O — 1=inlier (chi² ≤ finalChi2Threshold), 0=outlier. */
  finalInlierMask: Uint8Array;
  initialChi2Sum: number;
  finalChi2Sum: number;
  iterations: number;
  /** Threshold after the adaptive loop's doublings. */
  finalChi2Threshold: number;
  adaptiveDoublings: number;
  finalInlierCount: number;
  finalInlierRatio: number;
  P: number;
  L: number;
  O: number;
}

export interface BaModuleRaw {
  optimize(
    initPoses12Flat: Float64Array,
    initLandmarks3Flat: Float64Array,
    observationsFlat: Float64Array,
    fixedPoseIndicesFlat: Float64Array,
    kRowMajor: Float64Array,
    leftExt12: Float64Array,
    rightExt12: Float64Array,
    opts: Record<string, unknown>,
  ): BaResult;
}

export interface BaModule {
  raw: BaModuleRaw;
  /**
   * Run sparse Bundle Adjustment over `P` poses, `L` landmarks, and
   * `O` observations.
   *
   * Layouts:
   *   initPoses12Flat       — 12*P (row-major 3×4 per pose)
   *   initLandmarks3Flat    —  3*L
   *   observationsFlat      —  5*O — [poseIdx, lmIdx, u, v, isLeftImage]
   *   fixedPoseIndices      — pose ids to fix (typically [0])
   *   K                     — 9 entries [fx,0,cx, 0,fy,cy, 0,0,1]
   *   leftExt12 / rightExt12— 12 each (row-major 3×4) — book uses
   *                           cam_left_->pose() / cam_right_->pose()
   */
  optimize(
    initPoses12Flat: Float64Array,
    initLandmarks3Flat: Float64Array,
    observationsFlat: Float64Array,
    fixedPoseIndices: Float64Array,
    K: Float64Array,
    leftExt12: Float64Array,
    rightExt12: Float64Array,
    opts?: BaOptions,
  ): BaResult;
}

type EmscriptenFactory = (opts?: {
  locateFile?: (path: string, prefix: string) => string;
}) => Promise<BaModuleRaw>;

let cached: Promise<BaModule> | null = null;

export function loadBaWasm(variant: 'baseline' = 'baseline'): Promise<BaModule> {
  if (cached) return cached;
  const base = `/wasm/myslam_ba.${variant}`;
  cached = import(/* @vite-ignore */ `${base}.js`).then(
    async (mod: { default: EmscriptenFactory }) => {
      const raw = await mod.default({
        locateFile: (path: string) => {
          if (path.endsWith('.wasm')) return `${base}.wasm`;
          return `/wasm/${path}`;
        },
      });
      return {
        raw,
        optimize(initPoses12, initLandmarks3, obs, fixedPoseIdx, K, leftExt, rightExt, opts = {}) {
          const flat: Record<string, unknown> = {
            iterations: opts.iterations ?? 10,
            chi2Init: opts.chi2Init ?? 5.991,
            adaptiveRounds: opts.adaptiveRounds ?? 5,
            adaptiveInlierRatio: opts.adaptiveInlierRatio ?? 0.5,
            useRobustKernel: opts.useRobustKernel ?? true,
          };
          if (opts.huberDelta !== undefined) flat.huberDelta = opts.huberDelta;
          return raw.optimize(initPoses12, initLandmarks3, obs, fixedPoseIdx, K, leftExt, rightExt, flat);
        },
      };
    },
  );
  return cached;
}

/** Build a 9-entry row-major K = [fx,0,cx, 0,fy,cy, 0,0,1]. */
export function makeBaKMatrix(fx: number, fy: number, cx: number, cy: number): Float64Array {
  return new Float64Array([fx, 0, cx, 0, fy, cy, 0, 0, 1]);
}

/** 12-entry row-major 3×4 identity extrinsic. */
export const IDENTITY_EXT12 = new Float64Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]);

/**
 * Pure-translation 3×4 row-major extrinsic. KITTI rectified stereo right
 * camera is `[I | (-baseline, 0, 0)]` relative to the left frame, so
 * `makeRectifiedExt12(-baseline, 0, 0)` matches the book's `cam_right_->pose()`.
 */
export function makeRectifiedExt12(tx: number, ty: number, tz: number): Float64Array {
  return new Float64Array([1, 0, 0, tx, 0, 1, 0, ty, 0, 0, 1, tz]);
}

/** Pack a 3×3 R + 3×1 t into a row-major 3×4 [R|t]. */
export function packPose12(R: Float64Array | number[], t: Float64Array | number[]): Float64Array {
  const out = new Float64Array(12);
  out[0]  = R[0]; out[1]  = R[1]; out[2]  = R[2]; out[3]  = t[0];
  out[4]  = R[3]; out[5]  = R[4]; out[6]  = R[5]; out[7]  = t[1];
  out[8]  = R[6]; out[9]  = R[7]; out[10] = R[8]; out[11] = t[2];
  return out;
}

/** Identity pose (T_cw = I). */
export function identityPose12(): Float64Array {
  return new Float64Array(IDENTITY_EXT12);
}
