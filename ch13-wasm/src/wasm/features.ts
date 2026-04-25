// Loader for the Phase C / Step 3 + Step 4 features WASM module.
// Matches the Embind surface declared in
// wasm-src/myslam/bindings/bind_features.cpp.
//
// Detector enum is sourced from the WASM module's exported constants so that
// we never go out of sync with the C++ side. UI code should reference
// `mod.detectors.GFTT` etc. rather than hard-coding integers.

export type DetectorKey = 'GFTT' | 'Harris' | 'FAST' | 'ORB';

export interface DetectOptions {
  /** Max keypoints to keep. Default 150 (matches ch13's `num_features`). */
  maxFeatures?: number;
  /** GFTT/Harris quality threshold. Default 0.01. */
  qualityLevel?: number;
  /** Min spacing between corners in pixels. Default 20. */
  minDistance?: number;
  /** Neighborhood size used by GFTT/Harris. Default 3. */
  blockSize?: number;
  /** Harris detector free param k. Default 0.04. */
  harrisK?: number;
  /** FAST intensity threshold. Default 20. */
  fastThreshold?: number;
  /** FAST non-maximal suppression. Default true. */
  nonmaxSuppression?: boolean;
  /** ORB pyramid scale factor. Default 1.2. */
  orbScaleFactor?: number;
  /** ORB pyramid levels. Default 8. */
  orbNLevels?: number;
  /** Optional 1-channel uint8 mask of size w*h. 0 = exclude. */
  mask?: Uint8Array;
}

export interface TrackOptions {
  /** LK window edge length in pixels. ch13 uses 11. */
  winSize?: number;
  /** Pyramid levels. ch13 uses 3. */
  maxLevel?: number;
  /** Iteration cap. Default 30. */
  maxIter?: number;
  /** Convergence epsilon. Default 0.01. */
  eps?: number;
  /** Pass cv::OPTFLOW_USE_INITIAL_FLOW. Default false. */
  useInitialFlow?: boolean;
  /** Initial guesses, same 3-tuple shape as `prevPts`. Required when useInitialFlow=true. */
  initialPts?: Float64Array;
}

export interface FeaturesModuleRaw {
  /** Returns Float64Array [x0, y0, score0, x1, y1, score1, …]. */
  detectFeatures(
    gray: Uint8Array,
    w: number,
    h: number,
    algo: number,
    opts: DetectOptions,
  ): Float64Array;
  /** Returns Float64Array [x0', y0', status0, x1', y1', status1, …]. status==1 ⇔ tracked. */
  trackLK(
    prev: Uint8Array,
    curr: Uint8Array,
    w: number,
    h: number,
    prevPts: Float64Array,
    opts: TrackOptions,
  ): Float64Array;
  opencvVersion(): string;
  DETECTOR_GFTT: number;
  DETECTOR_HARRIS: number;
  DETECTOR_FAST: number;
  DETECTOR_ORB: number;
}

export interface FeaturesModule {
  /** Raw Embind handle — prefer the `detect`/`track` wrappers below. */
  raw: FeaturesModuleRaw;
  /** Map of UI-friendly names to the WASM-side enum integer. */
  detectors: Record<DetectorKey, number>;
  opencvVersion: string;
  detect(
    gray: Uint8Array,
    width: number,
    height: number,
    algo: DetectorKey,
    opts?: DetectOptions,
  ): Float64Array;
  track(
    prev: Uint8Array,
    curr: Uint8Array,
    width: number,
    height: number,
    prevPts: Float64Array,
    opts?: TrackOptions,
  ): Float64Array;
}

type EmscriptenFactory = (opts?: {
  locateFile?: (path: string, prefix: string) => string;
}) => Promise<FeaturesModuleRaw>;

let cached: Promise<FeaturesModule> | null = null;

export function loadFeaturesWasm(variant: 'baseline' = 'baseline'): Promise<FeaturesModule> {
  if (cached) return cached;
  const base = `/wasm/myslam_features.${variant}`;
  cached = import(/* @vite-ignore */ `${base}.js`).then(
    async (mod: { default: EmscriptenFactory }) => {
      const raw = await mod.default({
        locateFile: (path: string) => {
          if (path.endsWith('.wasm')) return `${base}.wasm`;
          return `/wasm/${path}`;
        },
      });
      const detectors: Record<DetectorKey, number> = {
        GFTT: raw.DETECTOR_GFTT,
        Harris: raw.DETECTOR_HARRIS,
        FAST: raw.DETECTOR_FAST,
        ORB: raw.DETECTOR_ORB,
      };
      return {
        raw,
        detectors,
        opencvVersion: raw.opencvVersion(),
        detect(gray, width, height, algo, opts = {}) {
          return raw.detectFeatures(gray, width, height, detectors[algo], opts);
        },
        track(prev, curr, width, height, prevPts, opts = {}) {
          return raw.trackLK(prev, curr, width, height, prevPts, opts);
        },
      };
    },
  );
  return cached;
}

/** ImageData (RGBA) → 1-channel grayscale Uint8Array using BT.601 luma weights. */
export function imageDataToGray(image: ImageData): Uint8Array {
  const out = new Uint8Array(image.width * image.height);
  const data = image.data;
  for (let i = 0, j = 0; i < data.length; i += 4, j++) {
    out[j] = (0.299 * data[i] + 0.587 * data[i + 1] + 0.114 * data[i + 2]) | 0;
  }
  return out;
}
