// Phase C body — verify the production features WASM binding on a synthetic
// frame pair (same renderFrame logic as scripts/gen-kitti-mini.ts and
// verify_cv.ts so the test has zero runtime deps beyond Node 22 built-ins).
//
// Run: node --experimental-strip-types wasm-src/spike/verify_features.ts
//
// Pass criteria:
//   * opencvVersion() returns "4.13.0"
//   * each detector (GFTT/Harris/FAST/ORB) returns ≥ 30 keypoints on frame 0
//   * GFTT mask exclusion drops keypoints inside a 100×100 hole
//   * trackLK frame 0 → frame 1 with GFTT seeds: ≥ 70% tracked, mean dx ≈ -14 px
//   * trackLK with useInitialFlow + ground-truth init keeps the result stable

import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

interface FeaturesModule {
  detectFeatures(
    gray: Uint8Array,
    w: number,
    h: number,
    algo: number,
    opts: Record<string, unknown>,
  ): Float64Array;
  trackLK(
    prev: Uint8Array,
    curr: Uint8Array,
    w: number,
    h: number,
    prevPts: Float64Array,
    opts: Record<string, unknown>,
  ): Float64Array;
  opencvVersion(): string;
  DETECTOR_GFTT: number;
  DETECTOR_HARRIS: number;
  DETECTOR_FAST: number;
  DETECTOR_ORB: number;
}

type ModuleFactory = (init?: { wasmBinary?: Uint8Array }) => Promise<FeaturesModule>;

const here = dirname(fileURLToPath(import.meta.url));

const W = 1226;
const H = 370;

function renderFrame(frameIdx: number, cameraIdx: 0 | 1): Uint8Array {
  const pixels = new Uint8Array(W * H);
  const forwardShift = frameIdx * 14;
  const stereoShift = cameraIdx === 0 ? 0 : 22;
  const markerXLeft = Math.floor(W * 0.48) - forwardShift;
  const markerXRight = markerXLeft - stereoShift;
  const markerY = Math.floor(H * 0.46);
  const m2xLeft = Math.floor(W * 0.25) - forwardShift;
  const m2xRight = m2xLeft - 11;
  const m2y = Math.floor(H * 0.72);
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const sx = x + stereoShift + forwardShift;
      let v = 40 + ((sx >> 1) & 0x3f);
      if ((sx + y) % 48 < 2) v += 35;
      if ((((sx >> 5) ^ (y >> 5)) & 1) === 1) v += 55;
      if (y === Math.floor(H * 0.5)) v = 230;
      const mx = cameraIdx === 0 ? markerXLeft : markerXRight;
      if (Math.abs(x - mx) < 5 && Math.abs(y - markerY) < 5) v = 255;
      const m2x = cameraIdx === 0 ? m2xLeft : m2xRight;
      if (Math.abs(x - m2x) < 4 && Math.abs(y - m2y) < 4) v = 250;
      pixels[y * W + x] = v < 0 ? 0 : v > 255 ? 255 : v | 0;
    }
  }
  return pixels;
}

function fail(msg: string): never {
  console.error(`[features] FAIL: ${msg}`);
  process.exit(1);
}

function countTriples(arr: Float64Array): number {
  return arr.length / 3;
}

async function main(): Promise<void> {
  const wasmJs = resolve(here, '..', '..', 'public', 'wasm', 'myslam_features.baseline.js');
  const wasmBin = resolve(here, '..', '..', 'public', 'wasm', 'myslam_features.baseline.wasm');
  const factory = ((await import(wasmJs)) as { default: ModuleFactory }).default;
  const wasmBinary = readFileSync(wasmBin);
  const Module = await factory({ wasmBinary });

  const version = Module.opencvVersion();
  console.log(`[features] OpenCV version: ${version}`);
  if (version !== '4.13.0') fail(`expected 4.13.0, got ${version}`);

  const frame0 = renderFrame(0, 0);

  const detectors = [
    { name: 'GFTT', algo: Module.DETECTOR_GFTT },
    { name: 'Harris', algo: Module.DETECTOR_HARRIS },
    { name: 'FAST', algo: Module.DETECTOR_FAST },
    { name: 'ORB', algo: Module.DETECTOR_ORB },
  ];
  const counts: Record<string, number> = {};
  for (const { name, algo } of detectors) {
    const t0 = performance.now();
    const out = Module.detectFeatures(frame0, W, H, algo, {
      maxFeatures: 200,
      qualityLevel: 0.01,
      minDistance: 20,
      blockSize: 3,
      fastThreshold: 20,
      nonmaxSuppression: true,
      orbScaleFactor: 1.2,
      orbNLevels: 8,
    });
    const dt = performance.now() - t0;
    const n = countTriples(out);
    counts[name] = n;
    console.log(`[features] ${name.padEnd(6)} → ${n} kps in ${dt.toFixed(1)} ms`);
    if (n < 30) fail(`${name}: expected ≥ 30 keypoints, got ${n}`);
  }

  // Mask test — exclude a 100×100 region around the central marker.
  // markerX in frame 0 = floor(1226*0.48) = 588, markerY = floor(370*0.46) = 170
  const mask = new Uint8Array(W * H).fill(255);
  const cx = 588;
  const cy = 170;
  for (let y = cy - 50; y < cy + 50; y++) {
    if (y < 0 || y >= H) continue;
    for (let x = cx - 50; x < cx + 50; x++) {
      if (x < 0 || x >= W) continue;
      mask[y * W + x] = 0;
    }
  }
  const masked = Module.detectFeatures(frame0, W, H, Module.DETECTOR_GFTT, {
    maxFeatures: 200,
    qualityLevel: 0.01,
    minDistance: 20,
    blockSize: 3,
    mask,
  });
  let inHole = 0;
  for (let i = 0; i < masked.length; i += 3) {
    const x = masked[i + 0];
    const y = masked[i + 1];
    if (x >= cx - 50 && x < cx + 50 && y >= cy - 50 && y < cy + 50) inHole++;
  }
  console.log(`[features] GFTT with 100×100 mask hole → ${countTriples(masked)} kps, ${inHole} inside hole`);
  if (inHole !== 0) fail(`mask exclusion broken: ${inHole} keypoints inside hole`);

  // LK left→right (use the synthetic horizontal pair: frame 0, camera 1) so
  // the disparity is the stereoShift constant (22 px → expected dx ≈ -22).
  const left0 = frame0;
  const right0 = renderFrame(0, 1);
  const seeds = Module.detectFeatures(left0, W, H, Module.DETECTOR_GFTT, {
    maxFeatures: 200,
    qualityLevel: 0.01,
    minDistance: 20,
  });
  const tLk = performance.now();
  const tracked = Module.trackLK(left0, right0, W, H, seeds, {
    winSize: 11,
    maxLevel: 3,
    maxIter: 30,
    eps: 0.01,
    useInitialFlow: false,
  });
  const dtLk = performance.now() - tLk;
  let ok = 0;
  let dxSum = 0;
  let dySum = 0;
  const n = countTriples(seeds);
  for (let i = 0; i < n; i++) {
    if (tracked[i * 3 + 2] > 0.5) {
      ok++;
      dxSum += tracked[i * 3 + 0] - seeds[i * 3 + 0];
      dySum += tracked[i * 3 + 1] - seeds[i * 3 + 1];
    }
  }
  const trackRate = ok / n;
  const meanDx = ok ? dxSum / ok : NaN;
  const meanDy = ok ? dySum / ok : NaN;
  console.log(
    `[features] LK left→right (no init) tracked ${ok}/${n} (${(trackRate * 100).toFixed(1)}%) in ${dtLk.toFixed(1)} ms`,
  );
  console.log(`[features]   mean shift: dx=${meanDx.toFixed(2)} dy=${meanDy.toFixed(2)} (expect ≈ -22, 0)`);
  if (trackRate < 0.7) fail(`LK trackRate ${(trackRate * 100).toFixed(1)}% < 70%`);
  if (Math.abs(meanDx + 22) > 3) fail(`LK mean dx ${meanDx.toFixed(2)} not near -22`);
  if (Math.abs(meanDy) > 1) fail(`LK mean dy ${meanDy.toFixed(2)} not near 0`);

  // useInitialFlow with GT-shifted seeds: should converge in even fewer iters.
  const initialPts = new Float64Array(seeds.length);
  for (let i = 0; i < n; i++) {
    initialPts[i * 3 + 0] = seeds[i * 3 + 0] - 22;
    initialPts[i * 3 + 1] = seeds[i * 3 + 1];
    initialPts[i * 3 + 2] = 1;
  }
  const trackedInit = Module.trackLK(left0, right0, W, H, seeds, {
    winSize: 11,
    maxLevel: 3,
    maxIter: 30,
    eps: 0.01,
    useInitialFlow: true,
    initialPts,
  });
  let okInit = 0;
  let dxSumInit = 0;
  for (let i = 0; i < n; i++) {
    if (trackedInit[i * 3 + 2] > 0.5) {
      okInit++;
      dxSumInit += trackedInit[i * 3 + 0] - seeds[i * 3 + 0];
    }
  }
  const meanDxInit = okInit ? dxSumInit / okInit : NaN;
  console.log(
    `[features] LK with useInitialFlow tracked ${okInit}/${n} (${((okInit / n) * 100).toFixed(1)}%), mean dx=${meanDxInit.toFixed(2)}`,
  );
  if (okInit < ok) fail(`useInitialFlow regressed: ${okInit} < ${ok}`);
  if (Math.abs(meanDxInit + 22) > 1.5) fail(`useInitialFlow dx ${meanDxInit.toFixed(2)} not near -22`);

  console.log('[features] ✅ all checks passed');
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
