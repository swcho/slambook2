// Loader for the Phase D / Step 5 triangulation WASM module.
// Matches the Embind surface declared in
// wasm-src/myslam/bindings/bind_triangulation.cpp.

export type TriangulationAlgo = 'LinearSVD' | 'Midpoint';

export interface TriangulateOptions {
  /** 'LinearSVD' (DLT, ch13 default) or 'Midpoint'. */
  algo?: TriangulationAlgo;
  /**
   * Quality threshold:
   *   - LinearSVD: σ4 / σ3 cutoff (default 0.01, matches ch13).
   *   - Midpoint: ray distance / mean depth cutoff (default 0.01).
   */
  qualityThreshold?: number;
  /**
   * If true, accept points that FAIL the quality check — literal
   * interpretation of algorithm.h's "포기" comment. PLAN §3 Step 5 학습
   * 포인트.
   */
  invertedReturn?: boolean;
}

export interface TriangulationModuleRaw {
  /**
   * Returns Float64Array stride 5 [x, y, z, qualityMetric, ok].
   * For input rows whose third element ≤ 0.5 (LK lost), the row is
   * (0, 0, 0, NaN, 0).
   */
  triangulate(
    leftPts: Float64Array,
    rightPts: Float64Array,
    kL: Float64Array,
    tL: Float64Array,
    kR: Float64Array,
    tR: Float64Array,
    opts: Record<string, unknown>,
  ): Float64Array;
  ALGO_LINEAR_SVD: number;
  ALGO_MIDPOINT: number;
}

export interface TriangulationModule {
  raw: TriangulationModuleRaw;
  algos: Record<TriangulationAlgo, number>;
  triangulate(
    leftPts: Float64Array,
    rightPts: Float64Array,
    kL: Float64Array,
    tL: Float64Array,
    kR: Float64Array,
    tR: Float64Array,
    opts?: TriangulateOptions,
  ): Float64Array;
}

type EmscriptenFactory = (opts?: {
  locateFile?: (path: string, prefix: string) => string;
}) => Promise<TriangulationModuleRaw>;

let cached: Promise<TriangulationModule> | null = null;

export function loadTriangulationWasm(
  variant: 'baseline' = 'baseline',
): Promise<TriangulationModule> {
  if (cached) return cached;
  const base = `/wasm/myslam_triangulation.${variant}`;
  cached = import(/* @vite-ignore */ `${base}.js`).then(
    async (mod: { default: EmscriptenFactory }) => {
      const raw = await mod.default({
        locateFile: (path: string) => {
          if (path.endsWith('.wasm')) return `${base}.wasm`;
          return `/wasm/${path}`;
        },
      });
      const algos: Record<TriangulationAlgo, number> = {
        LinearSVD: raw.ALGO_LINEAR_SVD,
        Midpoint: raw.ALGO_MIDPOINT,
      };
      return {
        raw,
        algos,
        triangulate(leftPts, rightPts, kL, tL, kR, tR, opts = {}) {
          const flat: Record<string, unknown> = {
            algo: opts.algo ? algos[opts.algo] : algos.LinearSVD,
            qualityThreshold:
              opts.qualityThreshold !== undefined ? opts.qualityThreshold : 0.01,
            invertedReturn: !!opts.invertedReturn,
          };
          return raw.triangulate(leftPts, rightPts, kL, tL, kR, tR, flat);
        },
      };
    },
  );
  return cached;
}

/**
 * Build the 3×4 row-major world→camera transform [R | t] for a KITTI rectified
 * pair. Both cameras share axes (R = identity); the right camera is shifted
 * so that the left camera centre is at the world origin.
 *
 *   tL = [I | 0]            (12 floats)
 *   tR = [I | (-baseline, 0, 0)]
 */
export function makeRectifiedT(translation: [number, number, number]): Float64Array {
  return new Float64Array([
    1, 0, 0, translation[0],
    0, 1, 0, translation[1],
    0, 0, 1, translation[2],
  ]);
}

export function makeK(fx: number, fy: number, cx: number, cy: number): Float64Array {
  return new Float64Array([fx, fy, cx, cy]);
}
