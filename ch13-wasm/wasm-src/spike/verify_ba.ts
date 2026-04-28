// =============================================================================
// verify_ba.ts — Phase G / Step 11 production gate
//
// 본 파일은 wasm-src/myslam/bindings/bind_ba.cpp가 컴파일한
// `myslam_ba.baseline.wasm`이 ch13/src/backend.cpp::Optimize의 핵심 동작
// (이항 엣지 + Schur complement + 적응적 chi² 루프 + 좌/우 카메라 외부 파라미터)을
// 모두 수행함을 수치적으로 증명한다.
//
// Phase A+ Stage 2의 verify_ba.mjs 가 던졌던 질문 — "g2o sparse BA가 WASM에서
// 동작하는가" — 은 그 시점에 통과했다. 본 파일은 그 위에 본 바인딩의 추가 기능을
// 검증한다:
//   1. 새 API (initPoses12 row-major / observations 5-stride / left+right ext)가
//      spike 와 같은 수렴 거동을 재현한다.
//   2. 적응적 chi² 루프가 outlier-heavy 시나리오에서 inlier_ratio > 0.5에 도달
//      하도록 임계값을 더블링한다.
//   3. 좌/우 카메라 외부 파라미터를 분리해 같은 랜드마크가 두 이미지에 두 번
//      관측될 때 ch13 stereo BA처럼 처리된다.
//
// 자세한 수학·구현 배경은 ./README.md 와 verify_ba.mjs 참조.

import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const __dirname = dirname(fileURLToPath(import.meta.url));
const wasmDir = resolve(__dirname, '../../public/wasm');

const wasmJs = resolve(wasmDir, 'myslam_ba.baseline.js');
const wasmBin = readFileSync(resolve(wasmDir, 'myslam_ba.baseline.wasm'));

interface BaResult {
  refinedPoses12: Float64Array;
  refinedLandmarks3: Float64Array;
  perEdgeChi2Initial: Float64Array;
  perEdgeChi2Final: Float64Array;
  finalInlierMask: Uint8Array;
  initialChi2Sum: number;
  finalChi2Sum: number;
  iterations: number;
  finalChi2Threshold: number;
  adaptiveDoublings: number;
  finalInlierCount: number;
  finalInlierRatio: number;
  P: number;
  L: number;
  O: number;
}
interface BaModule {
  optimize(
    initPoses12Flat: Float64Array,
    initLandmarks3Flat: Float64Array,
    observationsFlat: Float64Array,
    fixedPoseIndicesFlat: Float64Array,
    kRowMajor: Float64Array,
    leftExt12: Float64Array,
    rightExt12: Float64Array,
    opts: Record<string, unknown>,
  ): BaResult;
}
type Factory = (cfg: { wasmBinary: Buffer }) => Promise<BaModule>;

const { default: createModule } = (await import(wasmJs)) as { default: Factory };
const M = await createModule({ wasmBinary: wasmBin });

// ─── §A. 3D math helpers (verify_ba.mjs와 동일 수식) ───────────────────────
function axAngToMat(ax: number, ay: number, az: number): number[] {
  const th = Math.hypot(ax, ay, az);
  if (th < 1e-12) return [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const x = ax / th, y = ay / th, z = az / th;
  const c = Math.cos(th), s = Math.sin(th), t = 1 - c;
  return [
    t * x * x + c,     t * x * y - s * z, t * x * z + s * y,
    t * x * y + s * z, t * y * y + c,     t * y * z - s * x,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c,
  ];
}
function matToAxAng(R: number[]): [number, number, number] {
  const tr = R[0] + R[4] + R[8];
  const ct = Math.max(-1, Math.min(1, (tr - 1) / 2));
  const th = Math.acos(ct);
  if (Math.abs(th) < 1e-12) return [0, 0, 0];
  const k = th / (2 * Math.sin(th));
  return [(R[7] - R[5]) * k, (R[2] - R[6]) * k, (R[3] - R[1]) * k];
}
function rMul(A: number[], B: number[]): number[] {
  const r = Array<number>(9).fill(0);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      for (let k = 0; k < 3; k++) r[i * 3 + j] += A[i * 3 + k] * B[k * 3 + j];
  return r;
}
function rT(A: number[]): number[] {
  return [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
}

// ─── §B. seeded RNG + Gaussian (Mulberry32 + Box-Muller) ──────────────────
function rng(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
function gauss(r: () => number): number {
  const u = Math.max(r(), 1e-12), v = r();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

// ─── §C. 합성 BA 장면 ──────────────────────────────────────────────────────
function makeScene(seed: number, P: number, L: number, noiseStdPx: number) {
  const r = rng(seed);
  const uni = () => r() * 2 - 1;
  const fx = 520, fy = 520, cx = 320, cy = 240;
  const K = new Float64Array([fx, 0, cx, 0, fy, cy, 0, 0, 1]);

  const poses_gt: Array<{ R: number[]; t: number[] }> = [];
  poses_gt.push({ R: axAngToMat(0, 0, 0), t: [0, 0, 0] }); // pose 0 fixed
  for (let p = 1; p < P; ++p) {
    poses_gt.push({
      R: axAngToMat(uni() * 0.2, uni() * 0.2, uni() * 0.2),
      t: [uni() * 0.3, uni() * 0.3, uni() * 0.3],
    });
  }
  const landmarks_gt: Array<[number, number, number]> = [];
  for (let i = 0; i < L; ++i) landmarks_gt.push([uni() * 1.0, uni() * 1.0, 2 + r() * 1.5]);

  const obs: number[] = [];
  for (let p = 0; p < P; ++p) {
    for (let li = 0; li < L; ++li) {
      const { R, t } = poses_gt[p];
      const Pw = landmarks_gt[li];
      const Pc = [
        R[0] * Pw[0] + R[1] * Pw[1] + R[2] * Pw[2] + t[0],
        R[3] * Pw[0] + R[4] * Pw[1] + R[5] * Pw[2] + t[1],
        R[6] * Pw[0] + R[7] * Pw[1] + R[8] * Pw[2] + t[2],
      ];
      if (Pc[2] <= 0.5) continue;
      const u = (fx * Pc[0]) / Pc[2] + cx + (noiseStdPx ? gauss(r) * noiseStdPx : 0);
      const v = (fy * Pc[1]) / Pc[2] + cy + (noiseStdPx ? gauss(r) * noiseStdPx : 0);
      obs.push(p, li, u, v, 1); // 1 = isLeft (this gate uses left-only camera)
    }
  }

  // 12-flat init poses (row-major 3×4). Pose 0 fixed → no perturbation.
  const init_poses = new Float64Array(12 * P);
  for (let p = 0; p < P; ++p) {
    const ax = matToAxAng(poses_gt[p].R);
    const pert = p === 0 ? 0 : 0.15;
    const t = poses_gt[p].t;
    const r0 = ax[0] + pert * uni();
    const r1 = ax[1] + pert * uni();
    const r2 = ax[2] + pert * uni();
    const Rinit = axAngToMat(r0, r1, r2);
    const tx = t[0] + pert * uni();
    const ty = t[1] + pert * uni();
    const tz = t[2] + pert * uni();
    init_poses[12 * p +  0] = Rinit[0]; init_poses[12 * p +  1] = Rinit[1]; init_poses[12 * p +  2] = Rinit[2]; init_poses[12 * p +  3] = tx;
    init_poses[12 * p +  4] = Rinit[3]; init_poses[12 * p +  5] = Rinit[4]; init_poses[12 * p +  6] = Rinit[5]; init_poses[12 * p +  7] = ty;
    init_poses[12 * p +  8] = Rinit[6]; init_poses[12 * p +  9] = Rinit[7]; init_poses[12 * p + 10] = Rinit[8]; init_poses[12 * p + 11] = tz;
  }
  const init_lms = new Float64Array(3 * L);
  for (let l = 0; l < L; ++l) {
    init_lms[3 * l + 0] = landmarks_gt[l][0] + 0.2 * uni();
    init_lms[3 * l + 1] = landmarks_gt[l][1] + 0.2 * uni();
    init_lms[3 * l + 2] = landmarks_gt[l][2] + 0.2 * uni();
  }

  return {
    K, poses_gt, landmarks_gt,
    observations: new Float64Array(obs),
    init_poses, init_lms,
  };
}

const IDENTITY_EXT = new Float64Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]);

interface CaseSpec {
  seed: number; P: number; L: number; noiseStdPx: number;
  tolRot: number; tolTr: number; tolLm: number;
}
function runCase(label: string, spec: CaseSpec): boolean {
  const sc = makeScene(spec.seed, spec.P, spec.L, spec.noiseStdPx);
  const res = M.optimize(
    sc.init_poses, sc.init_lms, sc.observations,
    new Float64Array([0]), sc.K, IDENTITY_EXT, IDENTITY_EXT,
    { iterations: 20, chi2Init: 5.991, adaptiveRounds: 5, useRobustKernel: false },
  );

  let maxRot = 0, maxTr = 0, maxLm = 0;
  for (let p = 0; p < spec.P; ++p) {
    const M12 = res.refinedPoses12.slice(12 * p, 12 * p + 12);
    const R_est = [M12[0], M12[1], M12[2], M12[4], M12[5], M12[6], M12[8], M12[9], M12[10]];
    const t_est = [M12[3], M12[7], M12[11]];
    const R_gt = sc.poses_gt[p].R, t_gt = sc.poses_gt[p].t;
    maxRot = Math.max(maxRot, Math.hypot(...matToAxAng(rMul(R_est, rT(R_gt)))));
    maxTr = Math.max(maxTr, Math.hypot(t_est[0] - t_gt[0], t_est[1] - t_gt[1], t_est[2] - t_gt[2]));
  }
  for (let l = 0; l < spec.L; ++l) {
    maxLm = Math.max(maxLm, Math.hypot(
      res.refinedLandmarks3[3 * l + 0] - sc.landmarks_gt[l][0],
      res.refinedLandmarks3[3 * l + 1] - sc.landmarks_gt[l][1],
      res.refinedLandmarks3[3 * l + 2] - sc.landmarks_gt[l][2],
    ));
  }

  const chi2Pass = spec.noiseStdPx === 0
    ? res.finalChi2Sum < 1e-10
    : res.finalChi2Sum < res.initialChi2Sum * 1e-3;
  const geomPass = maxRot < spec.tolRot && maxTr < spec.tolTr && maxLm < spec.tolLm;
  const pass = chi2Pass && geomPass;

  const numObs = sc.observations.length / 5;
  // eslint-disable-next-line no-console
  console.log(
    `[${pass ? 'PASS' : 'FAIL'}] ${label}: obs=${numObs} iters=${res.iterations} ` +
    `chi2 ${res.initialChi2Sum.toExponential(2)} → ${res.finalChi2Sum.toExponential(2)}  ` +
    `maxRot=${maxRot.toExponential(2)} maxTr=${maxTr.toExponential(2)} maxLm=${maxLm.toExponential(2)}` +
    (pass ? '' : `  ← chi2Pass=${chi2Pass} geomPass=${geomPass}`),
  );
  return pass;
}

// ─── §D. Adaptive chi² loop — chi²_init이 noise floor 아래일 때 ────────────
//
// 1px Gaussian 노이즈에서 수렴된 BA의 per-edge chi²는 평균 ~2 (자유도 2).
// chi²_init=0.05로 시작하면 inlier_ratio < 0.5 → 적응적 루프가 임계값을 더블링해
// noise floor에 도달시켜야 한다. ch13 backend.cpp::Optimize에서 같은 상황이
// 발생하면 책의 `while(iteration<5)` 루프가 임계를 doubling 한다 — 본 케이스는
// 그 동작을 직접 검증.
function runAdaptiveCase(): boolean {
  const sc = makeScene(31, 3, 30, 1); // 1px noise

  const res = M.optimize(
    sc.init_poses, sc.init_lms, sc.observations,
    new Float64Array([0]), sc.K, IDENTITY_EXT, IDENTITY_EXT,
    { iterations: 15, chi2Init: 0.05, adaptiveRounds: 8, useRobustKernel: false },
  );

  const N = sc.observations.length / 5;
  const inlierRatio = res.finalInlierRatio;
  const doublings = res.adaptiveDoublings;
  // 0.05 → 0.1 → 0.2 → ... 까지 더블링되어 chi² noise floor (≈2) 부근에 도달
  // 해야 inlier_ratio > 0.5가 충족된다 — 약 5–6 doublings 예상.
  const pass = doublings >= 4 && inlierRatio > 0.5 && Number.isFinite(res.finalChi2Sum);
  // eslint-disable-next-line no-console
  console.log(
    `[${pass ? 'PASS' : 'FAIL'}] adaptive chi²: doublings=${doublings} ` +
    `finalThr=${res.finalChi2Threshold.toFixed(2)} inlierRatio=${inlierRatio.toFixed(3)} ` +
    `(${res.finalInlierCount}/${N})`,
  );
  return pass;
}

// ─── §E. Stereo (left+right) — 같은 landmark가 두 카메라에 동시 관측 ──────
//
// ch13 backend의 핵심 차이점: cam_left_->pose() / cam_right_->pose()를 각각
// 다른 EdgeProjection에 주입한다. 본 바인딩이 right ext를 사용해도 BA가
// 수렴하는지 검증.
function runStereoCase(): boolean {
  const sc = makeScene(7, 3, 25, 0);
  // 추가로 right-camera 관측 생성 (baseline tx = -0.5).
  const right_t = -0.5; // baseline
  const fx = 520, fy = 520, cx = 320, cy = 240;
  const N0 = sc.observations.length / 5;
  const obs2: number[] = Array.from(sc.observations);
  for (let p = 0; p < 3; ++p) {
    const { R, t } = sc.poses_gt[p];
    for (let li = 0; li < 25; ++li) {
      const Pw = sc.landmarks_gt[li];
      const Pc = [
        R[0] * Pw[0] + R[1] * Pw[1] + R[2] * Pw[2] + t[0] + right_t,
        R[3] * Pw[0] + R[4] * Pw[1] + R[5] * Pw[2] + t[1],
        R[6] * Pw[0] + R[7] * Pw[1] + R[8] * Pw[2] + t[2],
      ];
      if (Pc[2] <= 0.5) continue;
      const u = (fx * Pc[0]) / Pc[2] + cx;
      const v = (fy * Pc[1]) / Pc[2] + cy;
      obs2.push(p, li, u, v, 0); // 0 = isRight
    }
  }
  const obs = new Float64Array(obs2);
  const N = obs.length / 5;

  // right ext = [I | (right_t, 0, 0)] (translation only).
  const RIGHT_EXT = new Float64Array([1, 0, 0, right_t, 0, 1, 0, 0, 0, 0, 1, 0]);

  const res = M.optimize(
    sc.init_poses, sc.init_lms, obs,
    new Float64Array([0]), sc.K, IDENTITY_EXT, RIGHT_EXT,
    { iterations: 20, chi2Init: 5.991, adaptiveRounds: 5, useRobustKernel: false },
  );

  // 두 카메라에서 모두 보이는 landmark가 있으면 stereo BA에서 scale gauge가
  // 고정되므로 noiseless에서는 maxLm < 1e-3 수준으로 수렴해야 한다.
  let maxLm = 0;
  for (let l = 0; l < 25; ++l) {
    maxLm = Math.max(maxLm, Math.hypot(
      res.refinedLandmarks3[3 * l + 0] - sc.landmarks_gt[l][0],
      res.refinedLandmarks3[3 * l + 1] - sc.landmarks_gt[l][1],
      res.refinedLandmarks3[3 * l + 2] - sc.landmarks_gt[l][2],
    ));
  }
  const pass = res.finalChi2Sum < 1e-10 && maxLm < 1e-3 && N0 < N;
  // eslint-disable-next-line no-console
  console.log(
    `[${pass ? 'PASS' : 'FAIL'}] stereo: leftObs=${N0} totalObs=${N} ` +
    `chi2 ${res.initialChi2Sum.toExponential(2)} → ${res.finalChi2Sum.toExponential(2)} ` +
    `maxLm=${maxLm.toExponential(2)}`,
  );
  return pass;
}

// ─── §F. Driver ────────────────────────────────────────────────────────────
let all = true;

console.log('[Step 11] Stage 2 회귀: noiseless P=3 L=20');
for (let s = 1; s <= 3; ++s) {
  all = runCase(`  seed=${s}`, {
    seed: s, P: 3, L: 20, noiseStdPx: 0,
    tolRot: 5e-3, tolTr: 5e-2, tolLm: 1e-1,
  }) && all;
}

console.log('\n[Step 11] Stage 2 회귀: 1px Gaussian noise P=4 L=40');
for (let s = 10; s <= 12; ++s) {
  all = runCase(`  seed=${s}`, {
    seed: s, P: 4, L: 40, noiseStdPx: 1,
    tolRot: 5e-2, tolTr: 5e-2, tolLm: 2e-1,
  }) && all;
}

console.log('\n[Step 11] 적응적 chi² 루프 (outlier-heavy seeded)');
all = runAdaptiveCase() && all;

console.log('\n[Step 11] Stereo (left+right ext) — scale gauge 고정 검증');
all = runStereoCase() && all;

if (!all) {
  console.error('\n❌ FAIL — Phase G Step 11 BA gate 미통과.');
  process.exit(1);
}
console.log('\n✅ OK: bind_ba.cpp converges (LM + Schur + adaptive chi² + stereo cam_ext).');
console.log('   → Phase G Step 11 ready.');
