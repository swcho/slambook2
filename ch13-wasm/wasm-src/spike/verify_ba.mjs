// Phase A+ Stage 2 gate test: synthetic-GT bundle adjustment.
//
// Builds a 2-pose + 20-landmark scene, projects all landmarks under both
// poses, perturbs the init for all non-fixed poses and all landmarks, and
// asserts that g2o's sparse BA (SimplicialLLT via solver_eigen) recovers the
// ground truth. Validates:
//   - Binary edge Jacobian (pose + landmark)
//   - setMarginalized(true) Schur-complement pipeline
//   - Sparse linear solver under Emscripten (no CXSparse)

import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const __dirname = dirname(fileURLToPath(import.meta.url));
const wasmDir = resolve(__dirname, '../../public/wasm');
const { default: createModule } = await import(resolve(wasmDir, 'myslam_ba_spike.baseline.js'));
const M = await createModule({ wasmBinary: readFileSync(resolve(wasmDir, 'myslam_ba_spike.baseline.wasm')) });
const toArray = v => { const a = []; for (let i = 0; i < v.size(); ++i) a.push(v.get(i)); return a; };

// ---------- 3D helpers ----------
function axAngToMat(ax, ay, az) {
  const th = Math.hypot(ax, ay, az);
  if (th < 1e-12) return [1,0,0, 0,1,0, 0,0,1];
  const x=ax/th, y=ay/th, z=az/th, c=Math.cos(th), s=Math.sin(th), t=1-c;
  return [t*x*x+c, t*x*y-s*z, t*x*z+s*y,
          t*x*y+s*z, t*y*y+c, t*y*z-s*x,
          t*x*z-s*y, t*y*z+s*x, t*z*z+c];
}
function matToAxAng(R) {
  const tr = R[0]+R[4]+R[8];
  const ct = Math.max(-1, Math.min(1, (tr-1)/2));
  const th = Math.acos(ct);
  if (Math.abs(th) < 1e-12) return [0,0,0];
  const k = th/(2*Math.sin(th));
  return [(R[7]-R[5])*k, (R[2]-R[6])*k, (R[3]-R[1])*k];
}
function rMul(A,B){ const r=Array(9).fill(0); for(let i=0;i<3;i++)for(let j=0;j<3;j++)for(let k=0;k<3;k++)r[i*3+j]+=A[i*3+k]*B[k*3+j]; return r; }
function rT(A){ return [A[0],A[3],A[6],A[1],A[4],A[7],A[2],A[5],A[8]]; }

function rng(seed) {
  let a = seed >>> 0;
  return () => {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
function gauss(r) {
  const u = Math.max(r(), 1e-12), v = r();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

function makeScene(seed, numPoses, numLandmarks, noiseStdPx) {
  const r = rng(seed);
  const uni = () => r() * 2 - 1;
  const fx=520, fy=520, cx=320, cy=240;
  const K = [fx,0,cx, 0,fy,cy, 0,0,1];

  // Ground-truth poses. Pose 0 is Identity (the fixed gauge). Subsequent
  // poses are small motions from Identity — like a stereo rig or a tiny VO track.
  const poses_gt = [];  // [{R, t}]
  poses_gt.push({ R: axAngToMat(0, 0, 0), t: [0, 0, 0] });
  for (let p = 1; p < numPoses; ++p) {
    poses_gt.push({
      R: axAngToMat(uni()*0.2, uni()*0.2, uni()*0.2),
      t: [uni()*0.3, uni()*0.3, uni()*0.3],
    });
  }

  // Landmarks: in a volume in front of the cameras.
  const landmarks_gt = [];
  for (let i = 0; i < numLandmarks; ++i) {
    landmarks_gt.push([uni()*1.0, uni()*1.0, 2 + r()*1.5]);
  }

  // Observations: project every landmark from every pose (if Pc.z > 0.5).
  const observations = [];
  for (let p = 0; p < numPoses; ++p) {
    for (let li = 0; li < numLandmarks; ++li) {
      const { R, t } = poses_gt[p];
      const Pw = landmarks_gt[li];
      const Pc = [
        R[0]*Pw[0]+R[1]*Pw[1]+R[2]*Pw[2]+t[0],
        R[3]*Pw[0]+R[4]*Pw[1]+R[5]*Pw[2]+t[1],
        R[6]*Pw[0]+R[7]*Pw[1]+R[8]*Pw[2]+t[2],
      ];
      if (Pc[2] <= 0.5) continue;
      const u = fx*Pc[0]/Pc[2] + cx + (noiseStdPx ? gauss(r)*noiseStdPx : 0);
      const v = fy*Pc[1]/Pc[2] + cy + (noiseStdPx ? gauss(r)*noiseStdPx : 0);
      observations.push(p, li, u, v);
    }
  }

  // Init: perturb non-fixed poses (all except 0) and ALL landmarks.
  const init_poses = [];
  for (let p = 0; p < numPoses; ++p) {
    const ax = matToAxAng(poses_gt[p].R);
    const pert = (p === 0) ? 0 : 0.15;  // fixed pose stays at GT; others perturb
    init_poses.push(
      poses_gt[p].t[0] + pert*uni(), poses_gt[p].t[1] + pert*uni(), poses_gt[p].t[2] + pert*uni(),
      ax[0] + pert*uni(), ax[1] + pert*uni(), ax[2] + pert*uni(),
    );
  }
  const init_landmarks = [];
  for (const lm of landmarks_gt) {
    init_landmarks.push(lm[0] + 0.2*uni(), lm[1] + 0.2*uni(), lm[2] + 0.2*uni());
  }

  return { K, poses_gt, landmarks_gt, observations, init_poses, init_landmarks };
}

function runCase(label, { seed, numPoses, numLandmarks, noiseStdPx, tolRot, tolTr, tolLm }) {
  const sc = makeScene(seed, numPoses, numLandmarks, noiseStdPx);
  const res = M.solveBA({
    init_poses: sc.init_poses,
    init_landmarks: sc.init_landmarks,
    observations: sc.observations,
    K_row_major: sc.K,
    fixed_poses: [0],
    max_iters: 20,
  });

  const refinedPoses = toArray(res.refined_poses);
  const refinedLms = toArray(res.refined_landmarks);
  const numObs = sc.observations.length / 4;

  // Compute max rotation / translation / landmark error across all entities.
  let maxRot = 0, maxTr = 0, maxLm = 0;
  for (let p = 0; p < numPoses; ++p) {
    const M12 = refinedPoses.slice(12*p, 12*p + 12);  // row-major 3x4
    const R_est = [M12[0],M12[1],M12[2], M12[4],M12[5],M12[6], M12[8],M12[9],M12[10]];
    const t_est = [M12[3], M12[7], M12[11]];
    const R_gt = sc.poses_gt[p].R, t_gt = sc.poses_gt[p].t;
    maxRot = Math.max(maxRot, Math.hypot(...matToAxAng(rMul(R_est, rT(R_gt)))));
    maxTr  = Math.max(maxTr, Math.hypot(t_est[0]-t_gt[0], t_est[1]-t_gt[1], t_est[2]-t_gt[2]));
  }
  for (let i = 0; i < numLandmarks; ++i) {
    maxLm = Math.max(maxLm, Math.hypot(
      refinedLms[3*i+0]-sc.landmarks_gt[i][0],
      refinedLms[3*i+1]-sc.landmarks_gt[i][1],
      refinedLms[3*i+2]-sc.landmarks_gt[i][2]));
  }

  // For noiseless cases, chi² must drop below 1e-10 (machine precision-limited
  // convergence). For noisy cases, chi² must drop by at least 3 orders of
  // magnitude. We relax the geometric tolerance because monocular BA with a
  // single fixed pose has a residual scale gauge — the optimum reached can
  // differ from GT by a uniform scale + (with noise) small rotation.
  const chi2Pass = noiseStdPx === 0
    ? res.final_chi2 < 1e-10
    : res.final_chi2 < res.initial_chi2 * 1e-3;
  const geomPass = maxRot < tolRot && maxTr < tolTr && maxLm < tolLm;
  const pass = chi2Pass && geomPass;

  console.log(
    `[${pass ? 'PASS' : 'FAIL'}] ${label}: obs=${numObs} iters=${res.iterations} ` +
    `chi2 ${res.initial_chi2.toExponential(2)} → ${res.final_chi2.toExponential(2)}  ` +
    `maxRot=${maxRot.toExponential(2)} maxTr=${maxTr.toExponential(2)} maxLm=${maxLm.toExponential(2)}`
  );
  return pass;
}

let all = true;
console.log('Noiseless (chi² → 0 + geometric scale-gauge tolerance):');
for (let s = 1; s <= 3; ++s) {
  all &= runCase(`seed=${s} P=3 L=20`, {
    seed: s, numPoses: 3, numLandmarks: 20, noiseStdPx: 0,
    tolRot: 5e-3, tolTr: 5e-2, tolLm: 1e-1,
  });
}
console.log('\n1px Gaussian noise (3-orders chi² drop):');
for (let s = 10; s <= 12; ++s) {
  all &= runCase(`seed=${s} P=4 L=40`, {
    seed: s, numPoses: 4, numLandmarks: 40, noiseStdPx: 1,
    tolRot: 5e-2, tolTr: 5e-2, tolLm: 2e-1,
  });
}

if (!all) { console.error('FAIL: BA did not meet tolerance'); process.exit(1); }
console.log('\nOK: g2o BA spike (solver_eigen / SimplicialLLT) converges.');
