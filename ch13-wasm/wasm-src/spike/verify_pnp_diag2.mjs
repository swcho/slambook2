// Diagnostic 2: perturb GT pose by a small amount and see if LM converges back.
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const __dirname = dirname(fileURLToPath(import.meta.url));
const wasmDir = resolve(__dirname, '../../public/wasm');
const { default: createModule } = await import(resolve(wasmDir, 'myslam_pnp_spike.baseline.js'));
const M = await createModule({ wasmBinary: readFileSync(resolve(wasmDir, 'myslam_pnp_spike.baseline.wasm')) });
const toArray = v => { const a = []; for (let i = 0; i < v.size(); ++i) a.push(v.get(i)); return a; };

// GT pose + points with Pc.z all > 1 (so Jacobian is well-behaved).
const fx=520, fy=520, cx=320, cy=240;
const K=[fx,0,cx, 0,fy,cy, 0,0,1];
const theta=0.1;
const R=[Math.cos(theta),0,Math.sin(theta), 0,1,0, -Math.sin(theta),0,Math.cos(theta)];
const t=[0.1,-0.05,2.0];

const Pws=[[0.3,0.2,0.1],[-0.2,0.3,-0.1],[0.1,-0.3,0.2],[-0.4,-0.1,0.3],[0.4,0.0,-0.2],
           [0.0,0.2,0.4],[0.2,-0.2,-0.3],[-0.3,0.1,0.1],[0.3,0.3,0.3],[-0.1,-0.3,-0.4]];
const pts3=[], pts2=[];
for (const Pw of Pws) {
  const Pc=[R[0]*Pw[0]+R[1]*Pw[1]+R[2]*Pw[2]+t[0], R[3]*Pw[0]+R[4]*Pw[1]+R[5]*Pw[2]+t[1], R[6]*Pw[0]+R[7]*Pw[1]+R[8]*Pw[2]+t[2]];
  pts3.push(...Pw); pts2.push(fx*Pc[0]/Pc[2]+cx, fy*Pc[1]/Pc[2]+cy);
}

const tr = R[0]+R[4]+R[8];
const angle = Math.acos(Math.max(-1, Math.min(1, (tr-1)/2)));
const k = angle < 1e-9 ? 0 : angle/(2*Math.sin(angle));
const ax = [(R[7]-R[5])*k, (R[2]-R[6])*k, (R[3]-R[1])*k];

for (const delta of [0.01, 0.1, 0.5, 1.0, 2.0]) {
  // Perturb GT init by delta units in translation (all axes) and rotation (y-axis).
  const init6 = [t[0]+delta, t[1]+delta, t[2]+delta, ax[0], ax[1]+delta, ax[2]];
  const r = M.solvePnP({ points3d_flat: pts3, points2d_flat: pts2, K_row_major: K, init_pose6: init6, max_iters: 20 });
  const T = toArray(r.Tcw_row_major);
  const dt = Math.hypot(T[3]-t[0], T[7]-t[1], T[11]-t[2]);
  console.log(`perturb=${delta}: iters=${r.iterations} chi2=${r.final_chi2.toExponential(2)}  trErr=${dt.toExponential(2)}`);
}

// Check: does giving a 6-vec of zeros ([0,0,0,0,0,0]) equal identity?
console.log('\n--- 6-zero init vs empty init ---');
const r_empty = M.solvePnP({ points3d_flat: pts3, points2d_flat: pts2, K_row_major: K, init_pose6: [], max_iters: 0 });
const r_zero  = M.solvePnP({ points3d_flat: pts3, points2d_flat: pts2, K_row_major: K, init_pose6: [0,0,0,0,0,0], max_iters: 0 });
console.log('  empty init chi2:', r_empty.final_chi2);
console.log('  zero  init chi2:', r_zero.final_chi2);
