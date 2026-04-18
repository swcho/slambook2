// Phase A+ Stage 1 gate test: synthetic-GT PnP numeric validation.
//
// Starts from a perturbed-GT init (the regime a real SLAM frontend uses —
// ch13's Frontend::EstimateCurrentPose seeds PnP with relative_motion * last_pose)
// and asserts the WASM-compiled g2o optimizer recovers ground truth to machine
// precision under noiseless observations, and to sub-pixel precision under 1px
// Gaussian noise.

import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const __dirname = dirname(fileURLToPath(import.meta.url));
const wasmDir = resolve(__dirname, '../../public/wasm');
const { default: createModule } = await import(resolve(wasmDir, 'myslam_pnp_spike.baseline.js'));
const M = await createModule({ wasmBinary: readFileSync(resolve(wasmDir, 'myslam_pnp_spike.baseline.wasm')) });

const toArray = v => { const a = []; for (let i = 0; i < v.size(); ++i) a.push(v.get(i)); return a; };

// ---------- 3D math helpers (row-major) ----------------------------------
function axisAngleToMat(ax, ay, az) {
  const theta = Math.hypot(ax, ay, az);
  if (theta < 1e-12) return [1,0,0, 0,1,0, 0,0,1];
  const x = ax/theta, y = ay/theta, z = az/theta;
  const c = Math.cos(theta), s = Math.sin(theta), t = 1 - c;
  return [
    t*x*x + c,   t*x*y - s*z, t*x*z + s*y,
    t*x*y + s*z, t*y*y + c,   t*y*z - s*x,
    t*x*z - s*y, t*y*z + s*x, t*z*z + c,
  ];
}
function matToAxisAngle(R) {
  const trace = R[0] + R[4] + R[8];
  const cosTheta = Math.max(-1, Math.min(1, (trace - 1) / 2));
  const theta = Math.acos(cosTheta);
  if (Math.abs(theta) < 1e-12) return [0, 0, 0];
  const k = theta / (2 * Math.sin(theta));
  return [(R[7]-R[5])*k, (R[2]-R[6])*k, (R[3]-R[1])*k];
}
function rMul(A, B) {
  const r = new Array(9).fill(0);
  for (let i = 0; i < 3; ++i)
    for (let j = 0; j < 3; ++j)
      for (let k = 0; k < 3; ++k) r[i*3+j] += A[i*3+k] * B[k*3+j];
  return r;
}
function rT(A) { return [A[0],A[3],A[6], A[1],A[4],A[7], A[2],A[5],A[8]]; }

// ---------- deterministic RNG + synthetic scene --------------------------
function rng(seed) {
  let a = seed >>> 0;
  return () => {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
function gauss(rand) {
  const u = Math.max(rand(), 1e-12), v = rand();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
function makeCase(seed, N, noiseStdPx) {
  const r = rng(seed);
  const uni = () => r() * 2 - 1;
  // Ground truth pose — modest rotation, ~2m in front.
  const R_gt = axisAngleToMat(uni() * 0.3, uni() * 0.3, uni() * 0.3);
  const t_gt = [uni() * 0.2, uni() * 0.2, 2 + r() * 0.5];

  const fx = 520, fy = 520, cx = 320, cy = 240;
  const K = [fx, 0, cx, 0, fy, cy, 0, 0, 1];

  const pts3 = [];
  const pts2 = [];
  for (let i = 0; i < N; ++i) {
    const Pw = [uni() * 1.5, uni() * 1.5, uni() * 1.5];
    const Pc = [
      R_gt[0]*Pw[0] + R_gt[1]*Pw[1] + R_gt[2]*Pw[2] + t_gt[0],
      R_gt[3]*Pw[0] + R_gt[4]*Pw[1] + R_gt[5]*Pw[2] + t_gt[1],
      R_gt[6]*Pw[0] + R_gt[7]*Pw[1] + R_gt[8]*Pw[2] + t_gt[2],
    ];
    if (Pc[2] <= 0.5) { --i; continue; }
    const nu = noiseStdPx ? gauss(r) * noiseStdPx : 0;
    const nv = noiseStdPx ? gauss(r) * noiseStdPx : 0;
    pts3.push(...Pw);
    pts2.push(fx * Pc[0] / Pc[2] + cx + nu, fy * Pc[1] / Pc[2] + cy + nv);
  }

  // Init: perturb GT by ~0.3 in translation and ~0.1 rad in rotation — a
  // plausible relative_motion * last_pose prior for a SLAM frontend.
  const axang_gt = matToAxisAngle(R_gt);
  const init_pose6 = [
    t_gt[0] + uni() * 0.3, t_gt[1] + uni() * 0.3, t_gt[2] + uni() * 0.3,
    axang_gt[0] + uni() * 0.1, axang_gt[1] + uni() * 0.1, axang_gt[2] + uni() * 0.1,
  ];
  return { K, pts3, pts2, R_gt, t_gt, init_pose6 };
}

function runCase(label, opts) {
  const { K, pts3, pts2, R_gt, t_gt, init_pose6 } = makeCase(opts.seed, opts.N, opts.noiseStdPx);
  const res = M.solvePnP({ points3d_flat: pts3, points2d_flat: pts2, K_row_major: K, init_pose6, max_iters: 20 });
  const T = toArray(res.Tcw_row_major);
  const R_est = [T[0],T[1],T[2], T[4],T[5],T[6], T[8],T[9],T[10]];
  const t_est = [T[3], T[7], T[11]];
  const rotErr = Math.hypot(...matToAxisAngle(rMul(R_est, rT(R_gt))));
  const trErr = Math.hypot(t_est[0]-t_gt[0], t_est[1]-t_gt[1], t_est[2]-t_gt[2]);
  const pass = rotErr < opts.tolR && trErr < opts.tolT;
  console.log(
    `[${pass ? 'PASS' : 'FAIL'}] ${label}: iters=${res.iterations} ` +
    `chi2=${res.final_chi2.toExponential(2)} rotErr=${rotErr.toExponential(2)}rad ` +
    `trErr=${trErr.toExponential(2)}`
  );
  return pass;
}

let all = true;
console.log('Noiseless (tolR=1e-8, tolT=1e-8):');
for (let s = 1; s <= 5; ++s) all &= runCase(`seed=${s} N=40`, { seed: s, N: 40, noiseStdPx: 0, tolR: 1e-8, tolT: 1e-8 });
console.log('\n1px Gaussian noise (tolR=1e-2, tolT=5e-2):');
for (let s = 10; s <= 12; ++s) all &= runCase(`seed=${s} N=80`, { seed: s, N: 80, noiseStdPx: 1, tolR: 1e-2, tolT: 5e-2 });

if (!all) { console.error('FAIL'); process.exit(1); }
console.log('\nOK: g2o PnP spike converges from SLAM-style prior within tolerance.');
