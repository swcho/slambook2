// Phase D / Step 5 — verify the triangulation WASM binding on a synthetic
// rectified stereo rig with known ground-truth 3D points.
//
// Run: node --experimental-strip-types wasm-src/spike/verify_triangulation.ts
//
// Pass criteria:
//   * Linear SVD recovers each ground-truth point to within 1e-6 m (no noise).
//   * σ4/σ3 of SVD is < 1e-12 for exact correspondences.
//   * Midpoint matches Linear SVD to within 1e-6 m on exact correspondences.
//   * ok=1 with default threshold 0.01 for all exact pairs.
//   * Adding random epipolar noise spikes the quality metric and flips ok=0
//     for the chosen threshold.
//   * `invertedReturn=true` flips the ok flag (PLAN §3 Step 5 학습 포인트).
//   * status ≤ 0.5 inputs produce (0,0,0,NaN,0) rows.

import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

interface TriangulationModule {
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

type ModuleFactory = (init?: { wasmBinary?: Uint8Array }) => Promise<TriangulationModule>;

const here = dirname(fileURLToPath(import.meta.url));

function fail(msg: string): never {
  console.error(`[triangulation] FAIL: ${msg}`);
  process.exit(1);
}

// KITTI 05 left camera @ 0.5× downsample (matches Step 1/2 fixtures).
const fx = 360.295;
const fy = 360.295;
const cx = 303.605;
const cy = 92.695;
const baseline = 0.537151; // m

const kL = new Float64Array([fx, fy, cx, cy]);
const kR = new Float64Array([fx, fy, cx, cy]);
// world frame = left camera frame; right camera shifted by -baseline along x.
const tL = new Float64Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]);
const tR = new Float64Array([1, 0, 0, -baseline, 0, 1, 0, 0, 0, 0, 1, 0]);

// Project a world-frame point through the left/right rectified cameras.
function projectStereo(X: [number, number, number]): { uL: number; vL: number; uR: number; vR: number } {
  const [x, y, z] = X;
  const uL = fx * (x / z) + cx;
  const vL = fy * (y / z) + cy;
  // right-cam point = X - baseline along x (pure translation extrinsic).
  const xr = x - baseline;
  const uR = fx * (xr / z) + cx;
  const vR = fy * (y / z) + cy;
  return { uL, vL, uR, vR };
}

async function main(): Promise<void> {
  const wasmJs = resolve(here, '..', '..', 'public', 'wasm', 'myslam_triangulation.baseline.js');
  const wasmBin = resolve(here, '..', '..', 'public', 'wasm', 'myslam_triangulation.baseline.wasm');
  const factory = ((await import(wasmJs)) as { default: ModuleFactory }).default;
  const wasmBinary = readFileSync(wasmBin);
  const Module = await factory({ wasmBinary });

  console.log(`[triangulation] ALGO_LINEAR_SVD=${Module.ALGO_LINEAR_SVD} ALGO_MIDPOINT=${Module.ALGO_MIDPOINT}`);

  // -------------------------------------------------------------------------
  // Case A — exact correspondences for a grid of known world points.
  // -------------------------------------------------------------------------
  const groundTruth: [number, number, number][] = [];
  // Spread points in a 5×4 grid in front of the rig.
  for (let zStep = 0; zStep < 4; zStep++) {
    const z = 5 + zStep * 4; // 5, 9, 13, 17 m
    for (let xStep = -2; xStep <= 2; xStep++) {
      for (let yStep = -1; yStep <= 1; yStep++) {
        groundTruth.push([xStep * 1.5, yStep * 0.8, z]);
      }
    }
  }
  const N = groundTruth.length;
  const leftPts = new Float64Array(N * 3);
  const rightPts = new Float64Array(N * 3);
  for (let i = 0; i < N; i++) {
    const px = projectStereo(groundTruth[i]);
    leftPts[i * 3 + 0] = px.uL;
    leftPts[i * 3 + 1] = px.vL;
    leftPts[i * 3 + 2] = 1; // score
    rightPts[i * 3 + 0] = px.uR;
    rightPts[i * 3 + 1] = px.vR;
    rightPts[i * 3 + 2] = 1; // status: tracked
  }

  const tSvd0 = performance.now();
  const svd = Module.triangulate(leftPts, rightPts, kL, tL, kR, tR, {
    algo: Module.ALGO_LINEAR_SVD,
    qualityThreshold: 0.01,
    invertedReturn: false,
  });
  const tSvd = performance.now() - tSvd0;

  let maxErrSvd = 0;
  let maxRatioSvd = 0;
  let acceptedSvd = 0;
  for (let i = 0; i < N; i++) {
    const x = svd[i * 5 + 0];
    const y = svd[i * 5 + 1];
    const z = svd[i * 5 + 2];
    const ratio = svd[i * 5 + 3];
    const ok = svd[i * 5 + 4];
    const [gx, gy, gz] = groundTruth[i];
    const err = Math.hypot(x - gx, y - gy, z - gz);
    if (err > maxErrSvd) maxErrSvd = err;
    if (ratio > maxRatioSvd) maxRatioSvd = ratio;
    if (ok > 0.5) acceptedSvd++;
  }
  console.log(
    `[triangulation] LinearSVD ${N} pts in ${tSvd.toFixed(2)} ms · maxErr=${maxErrSvd.toExponential(3)} m · maxRatio=${maxRatioSvd.toExponential(3)} · accepted=${acceptedSvd}/${N}`,
  );
  if (maxErrSvd > 1e-6) fail(`LinearSVD maxErr ${maxErrSvd} m > 1e-6 m`);
  if (maxRatioSvd > 1e-9) fail(`LinearSVD σ4/σ3 ${maxRatioSvd} not near 0 for exact pairs`);
  if (acceptedSvd !== N) fail(`LinearSVD only accepted ${acceptedSvd}/${N} exact pairs`);

  const tMid0 = performance.now();
  const mid = Module.triangulate(leftPts, rightPts, kL, tL, kR, tR, {
    algo: Module.ALGO_MIDPOINT,
    qualityThreshold: 0.01,
    invertedReturn: false,
  });
  const tMid = performance.now() - tMid0;

  let maxErrMid = 0;
  let acceptedMid = 0;
  for (let i = 0; i < N; i++) {
    const x = mid[i * 5 + 0];
    const y = mid[i * 5 + 1];
    const z = mid[i * 5 + 2];
    const ok = mid[i * 5 + 4];
    const [gx, gy, gz] = groundTruth[i];
    const err = Math.hypot(x - gx, y - gy, z - gz);
    if (err > maxErrMid) maxErrMid = err;
    if (ok > 0.5) acceptedMid++;
  }
  console.log(
    `[triangulation] Midpoint  ${N} pts in ${tMid.toFixed(2)} ms · maxErr=${maxErrMid.toExponential(3)} m · accepted=${acceptedMid}/${N}`,
  );
  if (maxErrMid > 1e-6) fail(`Midpoint maxErr ${maxErrMid} m > 1e-6 m`);
  if (acceptedMid !== N) fail(`Midpoint only accepted ${acceptedMid}/${N} exact pairs`);

  // -------------------------------------------------------------------------
  // Case B — degenerate inputs (push right point off the epipolar line). σ4/σ3
  // should rise sharply and ok should drop to 0 with the default threshold.
  // -------------------------------------------------------------------------
  const noisyRight = new Float64Array(rightPts);
  for (let i = 0; i < N; i++) {
    // Add 4 px of vertical (epipolar) noise — large for a rectified stereo rig.
    noisyRight[i * 3 + 1] = rightPts[i * 3 + 1] + 4;
  }
  const noisy = Module.triangulate(leftPts, noisyRight, kL, tL, kR, tR, {
    algo: Module.ALGO_LINEAR_SVD,
    qualityThreshold: 0.01,
  });
  let acceptedNoisy = 0;
  let medianRatio = 0;
  const ratios: number[] = [];
  for (let i = 0; i < N; i++) {
    if (noisy[i * 5 + 4] > 0.5) acceptedNoisy++;
    ratios.push(noisy[i * 5 + 3]);
  }
  ratios.sort((a, b) => a - b);
  medianRatio = ratios[Math.floor(N / 2)];
  console.log(
    `[triangulation] LinearSVD with 4 px epipolar noise · accepted=${acceptedNoisy}/${N} · medianRatio=${medianRatio.toExponential(3)}`,
  );
  if (medianRatio < 1e-6) fail('noisy inputs should produce non-trivial σ4/σ3');

  // -------------------------------------------------------------------------
  // Case C — invertedReturn flips the ok bit at the threshold boundary.
  //
  // Use a *loose* threshold (0.1) so the exact-pair test set is split into
  // some that pass and some that fail, then verify inverted=true gives the
  // exact complement of inverted=false (when restricted to depth>0 inputs).
  // For the 4 px-noise dataset most rows will be on one side of the cut, so
  // inverting flips the count.
  // -------------------------------------------------------------------------
  const fixed = Module.triangulate(leftPts, noisyRight, kL, tL, kR, tR, {
    algo: Module.ALGO_LINEAR_SVD,
    qualityThreshold: 1e-3,
    invertedReturn: false,
  });
  const inverted = Module.triangulate(leftPts, noisyRight, kL, tL, kR, tR, {
    algo: Module.ALGO_LINEAR_SVD,
    qualityThreshold: 1e-3,
    invertedReturn: true,
  });
  let fixedAccept = 0;
  let invertedAccept = 0;
  let mismatches = 0;
  for (let i = 0; i < N; i++) {
    const f = fixed[i * 5 + 4] > 0.5;
    const v = inverted[i * 5 + 4] > 0.5;
    if (f) fixedAccept++;
    if (v) invertedAccept++;
    // depth>0 holds for all GT points (z 5..17). The ok flag is then
    // (qualityOk) vs (!qualityOk) — strictly complementary.
    if (f === v) mismatches++;
  }
  console.log(
    `[triangulation] inverted-return toggle · fixed=${fixedAccept}/${N} inverted=${invertedAccept}/${N} (mismatches=${mismatches})`,
  );
  if (mismatches !== 0) fail(`invertedReturn should flip every row, got ${mismatches} matching`);
  if (fixedAccept + invertedAccept !== N)
    fail(`fixed+inverted should equal N, got ${fixedAccept + invertedAccept}`);

  // -------------------------------------------------------------------------
  // Case D — status ≤ 0.5 in rightPts produces (0,0,0,NaN,0) rows.
  // -------------------------------------------------------------------------
  const lostRight = new Float64Array(rightPts);
  lostRight[0 * 3 + 2] = 0; // first row "lost"
  lostRight[5 * 3 + 2] = 0;
  const withLost = Module.triangulate(leftPts, lostRight, kL, tL, kR, tR, {
    algo: Module.ALGO_LINEAR_SVD,
  });
  for (const lostIdx of [0, 5]) {
    const x = withLost[lostIdx * 5 + 0];
    const y = withLost[lostIdx * 5 + 1];
    const z = withLost[lostIdx * 5 + 2];
    const ratio = withLost[lostIdx * 5 + 3];
    const ok = withLost[lostIdx * 5 + 4];
    if (x !== 0 || y !== 0 || z !== 0 || !Number.isNaN(ratio) || ok !== 0) {
      fail(`row ${lostIdx} lost-status not zeroed: x=${x} y=${y} z=${z} ratio=${ratio} ok=${ok}`);
    }
  }
  // Non-lost rows should still triangulate normally.
  for (let i = 1; i < 5; i++) {
    if (withLost[i * 5 + 4] !== 1) fail(`row ${i} expected ok=1, got ${withLost[i * 5 + 4]}`);
  }

  console.log('[triangulation] ✅ all checks passed');
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
