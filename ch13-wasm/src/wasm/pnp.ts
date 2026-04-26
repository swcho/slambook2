// Loader for the Phase E / Step 8 pose-only PnP WASM module.
// Mirrors the Embind surface declared in
// wasm-src/myslam/bindings/bind_pnp.cpp.

export interface PnPOptions {
  /** Outer rounds of LM + outlier reclassification. ch13 default = 4. */
  rounds?: number;
  /** LM iterations inside each round. ch13 default = 10. */
  iterPerRound?: number;
  /** chi² threshold for inlier classification. ch13 default = 5.991. */
  chi2Threshold?: number;
  /** Huber kernel δ. Default = sqrt(chi2Threshold). */
  huberDelta?: number;
  /**
   * After this round index (0-based), the Huber kernel is detached so the
   * optimizer chases inliers tightly. ch13 uses 2 (last two rounds).
   */
  removeRobustAfterRound?: number;
  /** Disable RobustKernel entirely. Default true (kernel on initially). */
  useRobustKernel?: boolean;
}

export interface PnPResult {
  /** 16 entries, row-major 4×4 Tcw. */
  Tcw_row_major: Float64Array;
  /** length N — 1 = inlier, 0 = outlier (final classification). */
  finalInlierMask: Uint8Array;
  /** length rounds * N — per-round inlier mask (animation source). */
  roundInlierMasks: Uint8Array;
  /** length rounds * N — per-edge chi² after each round's optimize. */
  roundChi2Values: Float64Array;
  /** length rounds — sum of inlier chi² per round. */
  roundChi2Sum: Float64Array;
  /** length rounds — inlier count per round. */
  roundInlierCount: Int32Array;
  /** length rounds — actual LM iterations consumed each round. */
  roundIters: Int32Array;
  /** Final inlier count (== last round entry of roundInlierCount). */
  totalInliers: number;
  /** optimizer.activeChi2() at the end. */
  finalChi2: number;
  rounds: number;
  N: number;
}

export interface PnPModuleRaw {
  estimatePose(
    points3dFlat: Float64Array,
    obs2dFlat: Float64Array,
    kRowMajor: Float64Array,
    initPose6: Float64Array,
    opts: Record<string, unknown>,
  ): PnPResult;
}

export interface PnPModule {
  raw: PnPModuleRaw;
  /**
   * Run pose-only PnP. Pass `initPose6 = new Float64Array(0)` for identity init.
   * `opts` defaults match ch13/src/frontend.cpp::EstimateCurrentPose.
   */
  estimatePose(
    points3d: Float64Array,
    obs2d: Float64Array,
    K: Float64Array,
    initPose6: Float64Array,
    opts?: PnPOptions,
  ): PnPResult;
}

type EmscriptenFactory = (opts?: {
  locateFile?: (path: string, prefix: string) => string;
}) => Promise<PnPModuleRaw>;

let cached: Promise<PnPModule> | null = null;

export function loadPnPWasm(variant: 'baseline' = 'baseline'): Promise<PnPModule> {
  if (cached) return cached;
  const base = `/wasm/myslam_pnp.${variant}`;
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
        estimatePose(points3d, obs2d, K, initPose6, opts = {}) {
          const flat: Record<string, unknown> = {
            rounds: opts.rounds ?? 4,
            iterPerRound: opts.iterPerRound ?? 10,
            chi2Threshold: opts.chi2Threshold ?? 5.991,
            removeRobustAfterRound: opts.removeRobustAfterRound ?? 2,
            useRobustKernel: opts.useRobustKernel ?? true,
          };
          if (opts.huberDelta !== undefined) flat.huberDelta = opts.huberDelta;
          return raw.estimatePose(points3d, obs2d, K, initPose6, flat);
        },
      };
    },
  );
  return cached;
}

/** Build a 9-entry row-major K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]. */
export function makeKMatrix(fx: number, fy: number, cx: number, cy: number): Float64Array {
  return new Float64Array([fx, 0, cx, 0, fy, cy, 0, 0, 1]);
}

/** Identity init for `initPose6` (zero-length flag). */
export function identityInit(): Float64Array {
  return new Float64Array(0);
}

/** Pack [tx, ty, tz, rx, ry, rz] (axis-angle) for `initPose6`. */
export function makeInitPose6(
  tx: number,
  ty: number,
  tz: number,
  rx: number,
  ry: number,
  rz: number,
): Float64Array {
  return new Float64Array([tx, ty, tz, rx, ry, rz]);
}
