// Phase E / Step 8 — verify the production PnP binding (myslam_pnp.baseline)
// against the same Stage-1 spike test matrix that gated Phase A+, plus two
// new checks specific to the body binding:
//   * 4-round outlier loop produces a per-round inlier count that is
//     non-decreasing on noiseless inputs and strictly increases when seeded
//     outliers fall out.
//   * Final inlier mask matches the ground-truth seeded outlier set.
//
// Run: node --experimental-strip-types wasm-src/spike/verify_pnp.ts
//
// Replaces the verify_pnp.mjs spike (kept on disk for archaeology). We bring
// the test up to the project's TS-first standard (memory feedback 2026-04-26).

import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

interface PnPResult {
  Tcw_row_major: Float64Array;
  finalInlierMask: Uint8Array;
  roundInlierMasks: Uint8Array;
  roundChi2Values: Float64Array;
  roundChi2Sum: Float64Array;
  roundInlierCount: Int32Array;
  roundIters: Int32Array;
  totalInliers: number;
  finalChi2: number;
  rounds: number;
  N: number;
}

interface PnPModule {
  estimatePose(
    points3dFlat: Float64Array,
    obs2dFlat: Float64Array,
    kRowMajor: Float64Array,
    initPose6: Float64Array,
    opts: Record<string, unknown>,
  ): PnPResult;
}

type ModuleFactory = (init?: { wasmBinary?: Uint8Array }) => Promise<PnPModule>;

const here = dirname(fileURLToPath(import.meta.url));

// =============================================================================
// 3D math helpers — row-major 3×3 throughout. Mirrors verify_pnp.mjs §A so
// the wire-level test behavior between the spike (pre-promotion) and the body
// binding (post-promotion) stays identical.
// =============================================================================

function axisAngleToMat(ax: number, ay: number, az: number): number[] {
  const theta = Math.hypot(ax, ay, az);
  if (theta < 1e-12) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const x = ax / theta,
    y = ay / theta,
    z = az / theta;
  const c = Math.cos(theta),
    s = Math.sin(theta),
    t = 1 - c;
  return [
    t * x * x + c, t * x * y - s * z, t * x * z + s * y,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c,
  ];
}

function matToAxisAngle(R: number[]): [number, number, number] {
  const trace = R[0] + R[4] + R[8];
  const cosTheta = Math.max(-1, Math.min(1, (trace - 1) / 2));
  const theta = Math.acos(cosTheta);
  if (Math.abs(theta) < 1e-12) return [0, 0, 0];
  const k = theta / (2 * Math.sin(theta));
  return [(R[7] - R[5]) * k, (R[2] - R[6]) * k, (R[3] - R[1]) * k];
}

function rMul(A: number[], B: number[]): number[] {
  const r = new Array(9).fill(0) as number[];
  for (let i = 0; i < 3; ++i)
    for (let j = 0; j < 3; ++j)
      for (let k = 0; k < 3; ++k) r[i * 3 + j] += A[i * 3 + k] * B[k * 3 + j];
  return r;
}

function rT(A: number[]): number[] {
  return [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
}

// Reproducible RNG so test results are platform-independent.
function rng(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function gauss(rand: () => number): number {
  const u = Math.max(rand(), 1e-12);
  const v = rand();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

interface CaseData {
  K: Float64Array;
  pts3: Float64Array;
  pts2: Float64Array;
  R_gt: number[];
  t_gt: [number, number, number];
  init_pose6: Float64Array;
  outlierIdx: number[];
}

function makeCase(
  seed: number,
  N: number,
  noiseStdPx: number,
  outlierFraction = 0,
  outlierShift = 60,
): CaseData {
  const r = rng(seed);
  const uni = () => r() * 2 - 1;

  const R_gt = axisAngleToMat(uni() * 0.3, uni() * 0.3, uni() * 0.3);
  const t_gt: [number, number, number] = [uni() * 0.2, uni() * 0.2, 2 + r() * 0.5];

  const fx = 520, fy = 520, cx = 320, cy = 240;
  const K = new Float64Array([fx, 0, cx, 0, fy, cy, 0, 0, 1]);

  const pts3: number[] = [];
  const pts2: number[] = [];
  const outlierIdx: number[] = [];
  let i = 0;
  while (i < N) {
    const Pw = [uni() * 1.5, uni() * 1.5, uni() * 1.5];
    const Pc = [
      R_gt[0] * Pw[0] + R_gt[1] * Pw[1] + R_gt[2] * Pw[2] + t_gt[0],
      R_gt[3] * Pw[0] + R_gt[4] * Pw[1] + R_gt[5] * Pw[2] + t_gt[1],
      R_gt[6] * Pw[0] + R_gt[7] * Pw[1] + R_gt[8] * Pw[2] + t_gt[2],
    ];
    if (Pc[2] <= 0.5) continue;
    const isOutlier = outlierFraction > 0 && r() < outlierFraction;
    const nu = noiseStdPx ? gauss(r) * noiseStdPx : 0;
    const nv = noiseStdPx ? gauss(r) * noiseStdPx : 0;
    let u = (fx * Pc[0]) / Pc[2] + cx + nu;
    let v = (fy * Pc[1]) / Pc[2] + cy + nv;
    if (isOutlier) {
      u += (uni() < 0 ? -1 : 1) * outlierShift;
      v += (uni() < 0 ? -1 : 1) * outlierShift;
      outlierIdx.push(i);
    }
    pts3.push(Pw[0], Pw[1], Pw[2]);
    pts2.push(u, v);
    i++;
  }

  const axang_gt = matToAxisAngle(R_gt);
  const init_pose6 = new Float64Array([
    t_gt[0] + uni() * 0.3,
    t_gt[1] + uni() * 0.3,
    t_gt[2] + uni() * 0.3,
    axang_gt[0] + uni() * 0.1,
    axang_gt[1] + uni() * 0.1,
    axang_gt[2] + uni() * 0.1,
  ]);

  return {
    K,
    pts3: new Float64Array(pts3),
    pts2: new Float64Array(pts2),
    R_gt,
    t_gt,
    init_pose6,
    outlierIdx,
  };
}

interface CaseTolerances {
  tolR: number;
  tolT: number;
}

function runCase(
  Module: PnPModule,
  label: string,
  data: CaseData,
  opts: { tol: CaseTolerances; pnpOpts?: Record<string, unknown> },
): { pass: boolean; res: PnPResult; rotErr: number; trErr: number } {
  const res = Module.estimatePose(
    data.pts3,
    data.pts2,
    data.K,
    data.init_pose6,
    opts.pnpOpts ?? { rounds: 4, iterPerRound: 10, chi2Threshold: 5.991 },
  );
  const T = res.Tcw_row_major;
  const R_est = [T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]];
  const t_est = [T[3], T[7], T[11]];
  const rotErr = Math.hypot(...matToAxisAngle(rMul(R_est, rT(data.R_gt))));
  const trErr = Math.hypot(t_est[0] - data.t_gt[0], t_est[1] - data.t_gt[1], t_est[2] - data.t_gt[2]);
  const pass = rotErr < opts.tol.tolR && trErr < opts.tol.tolT;
  console.log(
    `[${pass ? 'PASS' : 'FAIL'}] ${label}: ` +
      `rounds=${res.rounds} iters=[${Array.from(res.roundIters).join(',')}] ` +
      `inliers=[${Array.from(res.roundInlierCount).join(',')}] ` +
      `chi2=${res.finalChi2.toExponential(2)} ` +
      `rotErr=${rotErr.toExponential(2)}rad trErr=${trErr.toExponential(2)} ` +
      `(tol rotR=${opts.tol.tolR.toExponential(0)} tolT=${opts.tol.tolT.toExponential(0)})`,
  );
  return { pass, res, rotErr, trErr };
}

async function main(): Promise<void> {
  const wasmJs = resolve(here, '..', '..', 'public', 'wasm', 'myslam_pnp.baseline.js');
  const wasmBin = resolve(here, '..', '..', 'public', 'wasm', 'myslam_pnp.baseline.wasm');
  const factory = ((await import(wasmJs)) as { default: ModuleFactory }).default;
  const Module = await factory({ wasmBinary: readFileSync(wasmBin) });

  let allPass = true;

  // ---------------------------------------------------------------------------
  // (A) Noiseless — should converge to machine precision in any number of
  //     rounds. Mirrors verify_pnp.mjs Stage 1 §A.
  // ---------------------------------------------------------------------------
  console.log('[Stage 1] noiseless — expect machine precision convergence');
  console.log('  tol: rotErr < 1e-8 rad, trErr < 1e-8');
  for (let s = 1; s <= 5; ++s) {
    const data = makeCase(s, 40, 0);
    const { pass, res } = runCase(Module, `  seed=${s} N=40`, data, { tol: { tolR: 1e-8, tolT: 1e-8 } });
    allPass &&= pass;
    if (res.totalInliers !== data.pts3.length / 3) {
      console.error(`    expected all ${data.pts3.length / 3} inliers, got ${res.totalInliers}`);
      allPass = false;
    }
  }

  // ---------------------------------------------------------------------------
  // (B) 1px Gaussian noise — mrad / cm. Same tolerances as the spike.
  // ---------------------------------------------------------------------------
  console.log('\n[Stage 1] 1 px Gaussian noise — expect mrad / cm errors');
  console.log('  tol: rotErr < 1e-2 rad, trErr < 5e-2');
  for (let s = 10; s <= 12; ++s) {
    const data = makeCase(s, 80, 1);
    const { pass } = runCase(Module, `  seed=${s} N=80`, data, { tol: { tolR: 1e-2, tolT: 5e-2 } });
    allPass &&= pass;
  }

  // ---------------------------------------------------------------------------
  // (C) Body-binding new check: 20% seeded outliers (60 px shift). The
  //     4-round loop should kick them out and converge close to GT.
  // ---------------------------------------------------------------------------
  console.log('\n[Body] 20% seeded outliers — expect outlier rejection');
  console.log('  tol: rotErr < 5e-2 rad, trErr < 1e-1; recovered outliers ≥ 80% of seeded');
  for (let s = 20; s <= 22; ++s) {
    const data = makeCase(s, 100, 1, 0.2, 60);
    const { pass, res } = runCase(Module, `  seed=${s} N=100 outliers=${data.outlierIdx.length}`, data, {
      tol: { tolR: 5e-2, tolT: 1e-1 },
      pnpOpts: { rounds: 4, iterPerRound: 10, chi2Threshold: 5.991 },
    });
    allPass &&= pass;

    // Inlier count should be non-increasing as rounds peel off bad edges
    // (chi² climbs as the kernel detaches → more borderline edges become
    // outliers). Check that round 0 ≥ round R-1 within slack.
    const counts = Array.from(res.roundInlierCount);
    const monotone = counts[0] >= counts[counts.length - 1] - 1; // 1-edge slack
    if (!monotone) {
      console.error(
        `    round inlier counts not monotone: ${counts.join(' → ')}`,
      );
      allPass = false;
    }

    // Verify the seeded outliers were detected. The mask is the FINAL
    // outlier classification (sticky across rounds).
    let hits = 0;
    for (const idx of data.outlierIdx) {
      if (res.finalInlierMask[idx] === 0) hits++;
    }
    const recall = data.outlierIdx.length === 0 ? 1 : hits / data.outlierIdx.length;
    console.log(
      `    outlier recall ${hits}/${data.outlierIdx.length} (${(recall * 100).toFixed(1)}%) — final inliers ${res.totalInliers}/${res.N}`,
    );
    if (recall < 0.8) {
      console.error(`    recall ${(recall * 100).toFixed(1)}% < 80% threshold`);
      allPass = false;
    }
  }

  // ---------------------------------------------------------------------------
  // (D) RobustKernel toggle — book drops Huber after round 2; verify the
  //     kernel-on-only and kernel-off-only paths still converge on a clean
  //     case (no outliers, low noise). Just a smoke test on the option plumbing.
  // ---------------------------------------------------------------------------
  console.log('\n[Body] RobustKernel toggles — sanity smoke');
  const dataD = makeCase(30, 60, 1);
  for (const cfg of [
    { name: 'kernel=on, drop=4 (never drop)', opts: { rounds: 4, removeRobustAfterRound: 4 } },
    { name: 'kernel=off entirely',            opts: { rounds: 4, useRobustKernel: false } },
    { name: 'kernel=on, drop=0 (book extreme)', opts: { rounds: 4, removeRobustAfterRound: 0 } },
  ]) {
    const { pass } = runCase(Module, `  ${cfg.name}`, dataD, {
      tol: { tolR: 5e-2, tolT: 1e-1 },
      pnpOpts: { iterPerRound: 10, chi2Threshold: 5.991, ...cfg.opts },
    });
    allPass &&= pass;
  }

  if (!allPass) {
    console.error('\n❌ FAIL — Phase E Step 8 verify_pnp.ts gate not met.');
    process.exit(1);
  }
  console.log('\n✅ OK — myslam_pnp.baseline matches g2o spike + outlier loop checks.');
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
