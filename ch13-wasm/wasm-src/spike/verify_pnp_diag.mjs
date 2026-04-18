// Diagnostic: verify computeError is correct by starting at GT pose.
import { createRequire } from 'node:module';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const require = createRequire(import.meta.url);
const __dirname = dirname(fileURLToPath(import.meta.url));
const wasmDir = resolve(__dirname, '../../public/wasm');
const { default: createModule } = await import(resolve(wasmDir, 'myslam_pnp_spike.baseline.js'));
const M = await createModule({ wasmBinary: readFileSync(resolve(wasmDir, 'myslam_pnp_spike.baseline.wasm')) });

// Deterministic: known pose + points, project with known pose.
const fx = 520, fy = 520, cx = 320, cy = 240;
const K = [fx, 0, cx, 0, fy, cy, 0, 0, 1];

// Pose: rotation 0.1 rad around y, translation (0.1, -0.05, 2.0)
const theta = 0.1;
const R = [
  Math.cos(theta), 0, Math.sin(theta),
  0, 1, 0,
  -Math.sin(theta), 0, Math.cos(theta),
];
const t = [0.1, -0.05, 2.0];

// Use 10 points in [-0.5, 0.5]^3 so they're in front of camera after transform.
const pts3 = [];
const pts2 = [];
const Pws = [
  [0.3, 0.2, 0.1],  [-0.2, 0.3, -0.1], [0.1, -0.3, 0.2],
  [-0.4, -0.1, 0.3], [0.4, 0.0, -0.2], [0.0, 0.2, 0.4],
  [0.2, -0.2, -0.3], [-0.3, 0.1, 0.1], [0.3, 0.3, 0.3], [-0.1, -0.3, -0.4],
];
for (const Pw of Pws) {
  const Pc = [
    R[0]*Pw[0] + R[1]*Pw[1] + R[2]*Pw[2] + t[0],
    R[3]*Pw[0] + R[4]*Pw[1] + R[5]*Pw[2] + t[1],
    R[6]*Pw[0] + R[7]*Pw[1] + R[8]*Pw[2] + t[2],
  ];
  const u = fx*Pc[0]/Pc[2] + cx;
  const v = fy*Pc[1]/Pc[2] + cy;
  pts3.push(...Pw);
  pts2.push(u, v);
  console.log(`Pw=${Pw.map(x=>x.toFixed(2))} → Pc.z=${Pc[2].toFixed(3)} uv=(${u.toFixed(1)},${v.toFixed(1)})`);
}

// Embind VectorDouble helper — convert to a plain JS array.
const toArray = v => { const a = []; for (let i = 0; i < v.size(); ++i) a.push(v.get(i)); return a; };

// Experiment 1: start at Identity (large delta from GT)
console.log('\n--- Experiment 1: init=Identity, 0 iters ---');
let r1 = M.solvePnP({
  points3d_flat: pts3, points2d_flat: pts2,
  K_row_major: K, init_pose6: [], max_iters: 0,
});
console.log('  init chi2 =', r1.final_chi2);
console.log('  Tcw[0:4] =', toArray(r1.Tcw_row_major).slice(0, 4).map(x=>x.toFixed(4)));

// Experiment 2: 1 LM iter from Identity
console.log('\n--- Experiment 2: init=Identity, 1 iter ---');
let r2 = M.solvePnP({
  points3d_flat: pts3, points2d_flat: pts2,
  K_row_major: K, init_pose6: [], max_iters: 1,
});
console.log('  iter=1 chi2 =', r2.final_chi2);

// Experiment 3: 20 iters from Identity
console.log('\n--- Experiment 3: init=Identity, 20 iters ---');
let r3 = M.solvePnP({
  points3d_flat: pts3, points2d_flat: pts2,
  K_row_major: K, init_pose6: [], max_iters: 20,
});
console.log('  iter=20 chi2 =', r3.final_chi2);
const T3 = toArray(r3.Tcw_row_major);
console.log('  Tcw =\n   ', [0,4,8,12].map(i => T3.slice(i,i+4).map(x=>x.toFixed(4)).join(' ')).join('\n    '));
console.log('  expected Tcw =\n   ',
  `${R[0].toFixed(4)} ${R[1].toFixed(4)} ${R[2].toFixed(4)} ${t[0].toFixed(4)}\n    ` +
  `${R[3].toFixed(4)} ${R[4].toFixed(4)} ${R[5].toFixed(4)} ${t[1].toFixed(4)}\n    ` +
  `${R[6].toFixed(4)} ${R[7].toFixed(4)} ${R[8].toFixed(4)} ${t[2].toFixed(4)}`);

// Experiment 4: init AT ground truth — chi2 should be ~0
console.log('\n--- Experiment 4: init=GT pose, 0 iters (validates computeError) ---');
// Convert R+t to 6-vec (axis-angle + translation).
const tr = R[0] + R[4] + R[8];
const angle = Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2)));
const k = angle < 1e-9 ? 0 : angle / (2 * Math.sin(angle));
const ax = [(R[7]-R[5])*k, (R[2]-R[6])*k, (R[3]-R[1])*k];
let r4 = M.solvePnP({
  points3d_flat: pts3, points2d_flat: pts2,
  K_row_major: K, init_pose6: [t[0], t[1], t[2], ax[0], ax[1], ax[2]], max_iters: 0,
});
console.log('  init chi2 at GT =', r4.final_chi2, '(should be ~0)');
