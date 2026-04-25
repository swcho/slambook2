// Phase C spike Q4 — verify the cv_spike WASM module on a synthetic frame
// pair generated with the same logic as scripts/gen-kitti-mini.ts. We
// regenerate the raw grayscale bytes directly (no PNG encode/decode) so the
// spike has zero runtime deps beyond Node 22's built-ins.
//
// Run: node --experimental-strip-types wasm-src/spike/verify_cv.ts
//
// Pass criteria:
//   1. opencvVersion() returns "4.13.0"
//   2. detectGFTT on frame 0 returns ≥ 50 corners
//   3. trackLK frame 0 → frame 1 has ≥ 70% status==1
//   4. mean dx of tracked points is ≈ -14 px (the forwardShift in renderFrame)
//      with tolerance ±2 px

import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

interface CvSpikeModule {
  detectGFTT(
    gray: Uint8Array,
    w: number,
    h: number,
    maxFeatures: number,
    quality: number,
    minDist: number,
  ): Float64Array;
  trackLK(
    prev: Uint8Array,
    curr: Uint8Array,
    w: number,
    h: number,
    pts: Float64Array,
  ): Float64Array;
  opencvVersion(): string;
}

type ModuleFactory = (init?: { wasmBinary?: Uint8Array }) => Promise<CvSpikeModule>;

const here = dirname(fileURLToPath(import.meta.url));

const W = 1226;
const H = 370;

// Mirror scripts/gen-kitti-mini.ts::renderFrame so we don't have to decode the
// PNG fixtures from disk. This is the same scene the rest of ch13-wasm renders
// in the browser.
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
  console.error(`[cv_spike] FAIL: ${msg}`);
  process.exit(1);
}

async function main(): Promise<void> {
  const wasmJs = resolve(here, '..', '..', 'public', 'wasm', 'myslam_cv_spike.baseline.js');
  const wasmBin = resolve(here, '..', '..', 'public', 'wasm', 'myslam_cv_spike.baseline.wasm');
  const factory = ((await import(wasmJs)) as { default: ModuleFactory }).default;
  const wasmBinary = readFileSync(wasmBin);
  const Module = await factory({ wasmBinary });

  const version = Module.opencvVersion();
  console.log(`[cv_spike] OpenCV version: ${version}`);
  if (version !== '4.13.0') fail(`expected 4.13.0, got ${version}`);

  const frame0 = renderFrame(0, 0);
  const frame1 = renderFrame(1, 0);

  // Q4-a: GFTT on frame 0
  const t0 = performance.now();
  const corners = Module.detectGFTT(frame0, W, H, 200, 0.01, 20);
  const tDetect = performance.now() - t0;
  const numCorners = corners.length / 2;
  console.log(`[cv_spike] GFTT detected ${numCorners} corners in ${tDetect.toFixed(1)} ms`);
  if (numCorners < 50) fail(`expected ≥ 50 corners, got ${numCorners}`);

  // Q4-b: LK frame 0 → frame 1
  const t1 = performance.now();
  const tracked = Module.trackLK(frame0, frame1, W, H, corners);
  const tTrack = performance.now() - t1;

  let trackedCount = 0;
  let dxSum = 0;
  let dySum = 0;
  for (let i = 0; i < numCorners; i++) {
    const status = tracked[i * 3 + 2];
    if (status > 0.5) {
      trackedCount++;
      const x0 = corners[i * 2 + 0];
      const y0 = corners[i * 2 + 1];
      const x1 = tracked[i * 3 + 0];
      const y1 = tracked[i * 3 + 1];
      dxSum += x1 - x0;
      dySum += y1 - y0;
    }
  }
  const trackRate = trackedCount / numCorners;
  const meanDx = trackedCount > 0 ? dxSum / trackedCount : NaN;
  const meanDy = trackedCount > 0 ? dySum / trackedCount : NaN;
  console.log(
    `[cv_spike] LK tracked ${trackedCount}/${numCorners} (${(trackRate * 100).toFixed(1)}%) in ${tTrack.toFixed(1)} ms`,
  );
  console.log(
    `[cv_spike] mean shift: dx=${meanDx.toFixed(2)} dy=${meanDy.toFixed(2)} (expected dx ≈ -14, dy ≈ 0)`,
  );

  if (trackRate < 0.7) fail(`expected ≥ 70% tracked, got ${(trackRate * 100).toFixed(1)}%`);
  if (Math.abs(meanDx + 14) > 2) fail(`expected mean dx ≈ -14, got ${meanDx.toFixed(2)}`);
  if (Math.abs(meanDy) > 1) fail(`expected mean dy ≈ 0, got ${meanDy.toFixed(2)}`);

  console.log('[cv_spike] ✅ all gates passed (Q1~Q4)');
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
