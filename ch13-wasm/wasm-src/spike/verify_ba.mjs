// =============================================================================
// verify_ba.mjs — Phase A+ Stage 2 Gate Test
//
// 이 스크립트는 `bind_ba_spike.cpp`가 컴파일한 `myslam_ba_spike.baseline.wasm`
// 이 **g2o 기반 Bundle Adjustment**를 Emscripten/WASM 환경에서 수행할 수 있는지
// **수치적으로 증명**한다. Gate 통과 여부는 exit code 하나로 판정: 0 = PASS,
// 1 = FAIL.
//
// Stage 1 (verify_pnp.mjs)은 "g2o가 WASM에서 링크되고 조그만 LM이 수렴하는가"를
// 묻는다. Stage 2인 본 파일은 더 나아가 **sparse 선형대수 + 이항 엣지 + Schur
// complement**까지 동작하는지 검증한다. 이는 Phase G(Step 11 Bundle Adjustment)의
// 축소판이므로, 본 테스트가 통과한다는 것은 Phase G의 핵심 기술 스택이 살아 있다
// 는 강한 증거다.
//
// 자세한 수학·구현 배경은 ./README.md 참조. 아래는 "본 파일이 주장하는 사양"을
// 자기 완결적으로 기술한다.
//
// -----------------------------------------------------------------------------
// 1. 문제 정의
// -----------------------------------------------------------------------------
// 카메라 포즈 집합 $\{T_p\}_{p=0}^{P-1}$과 월드 좌표계의 3D 랜드마크 집합
// $\{P_w^l\}_{l=0}^{L-1}$, 그리고 관측 튜플 집합 $\mathcal{O} = \{(p_k, l_k, z_k)\}$
// 가 주어진다. 비선형 최소제곱:
//
//   $$ \{T_p^*, P_w^{l*}\} = \arg\min \sum_{(p, l, z) \in \mathcal{O}}
//        \| z - \pi(K\, T_p\, P_w^l) \|^2 $$
//
// 이를 Gauss-Newton / Levenberg-Marquardt로 풀되, landmark 변수는 **Schur
// complement**로 소거하여 각 반복에서 작은 pose-block 선형시스템만 풀어도 되도록
// g2o의 `setMarginalized(true)` 관용구를 사용한다.
//
// -----------------------------------------------------------------------------
// 2. Gauge 자유도 처리
// -----------------------------------------------------------------------------
// BA는 문제 본질상 다음 7 DOF의 **gauge 자유도**를 가진다:
//   - 전체 좌표계 평행이동 (3 DOF)
//   - 전체 좌표계 회전 (3 DOF)
//   - 단안 시스템에서의 scale (1 DOF)
//
// 본 테스트는 **pose 0을 $T_0 = I$로 고정** (`fixed_poses = [0]`)하여 앞의 6 DOF만
// 제거한다. scale은 자유. 따라서 noiseless라도 **GT와 최대 ~수 % 스케일 드리프트**
// 가 남을 수 있다 — 이는 버그가 아니라 수학적 필연. 실제 ch13 파이프라인은 stereo
// baseline이 scale을 고정하므로 Phase G에서는 이 문제가 발생하지 않는다.
//
// -----------------------------------------------------------------------------
// 3. 합성 데이터 생성 규약
// -----------------------------------------------------------------------------
// 같은 Mulberry32 RNG로 재현 가능한 장면을 만든다:
//   - Pose 0: Identity (gauge fix용 고정 포즈).
//   - Pose 1, 2, ...: axis-angle $|\phi| \le 0.2$ rad, translation $\in [-0.3, 0.3]^3$.
//   - Landmark: $(x, y) \in [-1, 1]^2$, $z \in [2, 3.5]$ (모든 pose에서 앞쪽에 보일
//     확률을 높이기 위해).
//   - 모든 (pose, landmark) 조합에 대해 $P_c.z > 0.5$일 때만 관측을 만든다.
//   - 픽셀 노이즈는 $\mathcal{N}(0, \sigma^2)$ Gaussian을 $u, v$ 각각에 독립 추가.
//
// -----------------------------------------------------------------------------
// 4. 초기치 규약
// -----------------------------------------------------------------------------
//   - Fixed pose (id=0): GT 그대로 (섭동 0).
//   - Free poses (id ≥ 1): translation·axis-angle 모두 GT ± 0.15 균일 섭동.
//   - Landmarks: GT ± 0.2 균일 섭동.
//
// -----------------------------------------------------------------------------
// 5. 판정 기준 — chi² + geometric error를 **동시에** 요구
// -----------------------------------------------------------------------------
// 각 케이스는 다음 두 조건을 **모두** 만족해야 PASS:
//
//   (a) chi² 수렴:
//       - Noiseless: $\chi^2_{final} < 10^{-10}$
//         → 거의 machine precision 수렴. scale gauge 외의 모든 에러가 제거됨.
//       - Noisy:     $\chi^2_{final} < \chi^2_{initial} \cdot 10^{-3}$
//         → 최소 3자릿수 이상 감소 (관측 수가 많으면 noise-floor가 더 높아짐).
//
//   (b) 기하 오차 (max over all poses / landmarks):
//       - maxRot  < tolRot   (axis-angle 크기의 최대)
//       - maxTr   < tolTr    (pose translation 유클리드 거리의 최대)
//       - maxLm   < tolLm    (landmark 위치 오차의 최대)
//
//       모노큘러 scale gauge가 남기 때문에 noiseless에서도 **절대 오차**는 0이 되지
//       않음. tolerance를 5e-3 (rot) / 5e-2 (tr) / 1e-1 (lm)로 놓는 이유.
//
// -----------------------------------------------------------------------------
// 6. 테스트 매트릭스
// -----------------------------------------------------------------------------
// (A) Noiseless × 3 seeds, P=3 poses, L=20 landmarks
//     기대값: chi² 20+자릿수 감소, scale drift 수 % 이내.
//
// (B) 1 px Gaussian × 3 seeds, P=4 poses, L=40 landmarks
//     기대값: chi² 3+자릿수 감소, noise floor $\approx 2 \cdot N_{obs}$ (자유도 당
//             단위 분산 잔차).
//
// -----------------------------------------------------------------------------
// 7. 통과 기준이 무너진다면 무엇을 의심할지
// -----------------------------------------------------------------------------
//   - chi²는 감소하나 landmark error만 큼 → Landmark Jacobian `_jacobianOplusXj`
//                                            유도식의 곱셈 순서 (`R * ...` vs
//                                            `... * R^\top`).
//   - Noiseless chi²가 10⁻¹⁰보다 크게 정체 → Schur complement 미적용 의심
//                                              (`setMarginalized(true)` 누락 등).
//   - LM iters가 항상 max에 도달 + chi² 정체 → linear solver backend 문제.
//                                              `LinearSolverEigen`이 실제로 sparse
//                                              경로인지 확인하라 (SimplicialLLT).
//   - 모든 케이스가 같은 수치로 실패        → RNG seed 전파 문제 (rng(seed) 위치).
// =============================================================================

import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const __dirname = dirname(fileURLToPath(import.meta.url));
const wasmDir = resolve(__dirname, '../../public/wasm');
const { default: createModule } = await import(resolve(wasmDir, 'myslam_ba_spike.baseline.js'));
const M = await createModule({ wasmBinary: readFileSync(resolve(wasmDir, 'myslam_ba_spike.baseline.wasm')) });

const toArray = v => { const a = []; for (let i = 0; i < v.size(); ++i) a.push(v.get(i)); return a; };

// =============================================================================
// §A. 3D 수학 도우미 (verify_pnp.mjs와 동일한 수식/구현)
// =============================================================================

/**
 * Rodrigues 공식: axis-angle → row-major 3×3 회전 행렬.
 *
 * $$ R = I + \sin\theta\, \hat{k}^{\wedge} + (1 - \cos\theta)\, (\hat{k}^{\wedge})^2 $$
 */
function axAngToMat(ax, ay, az) {
  const th = Math.hypot(ax, ay, az);
  if (th < 1e-12) return [1,0,0, 0,1,0, 0,0,1];
  const x=ax/th, y=ay/th, z=az/th, c=Math.cos(th), s=Math.sin(th), t=1-c;
  return [t*x*x+c, t*x*y-s*z, t*x*z+s*y,
          t*x*y+s*z, t*y*y+c, t*y*z-s*x,
          t*x*z-s*y, t*y*z+s*x, t*z*z+c];
}

/**
 * 회전 행렬 → axis-angle.
 *
 * $$ \theta = \arccos\left(\frac{\mathrm{tr}(R) - 1}{2}\right) $$
 */
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

// =============================================================================
// §B. 재현 가능한 난수 (Mulberry32) + Gaussian
// =============================================================================

function rng(seed) {
  let a = seed >>> 0;
  return () => {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Box–Muller: $Z = \sqrt{-2 \ln U_1} \cos(2\pi U_2) \sim \mathcal{N}(0,1)$. */
function gauss(r) {
  const u = Math.max(r(), 1e-12), v = r();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

// =============================================================================
// §C. 합성 BA 장면 생성 (§3, §4 규약 구현)
// =============================================================================

/**
 * BA 한 케이스. Pose 0은 Identity로 고정하고, 나머지 pose와 모든 landmark를 섭동
 * 시킨 init을 만든다.
 *
 * 관측 `observations`는 **flat 배열**로 `[poseIdx, landmarkIdx, u, v]`가 반복됨
 * (`bind_ba_spike.cpp::solveBA`의 입력 규약).
 */
function makeScene(seed, numPoses, numLandmarks, noiseStdPx) {
  const r = rng(seed);
  const uni = () => r() * 2 - 1;
  const fx=520, fy=520, cx=320, cy=240;
  const K = [fx,0,cx, 0,fy,cy, 0,0,1];

  // ── Ground-truth 포즈
  const poses_gt = [];
  poses_gt.push({ R: axAngToMat(0, 0, 0), t: [0, 0, 0] });  // pose 0 = Identity
  for (let p = 1; p < numPoses; ++p) {
    poses_gt.push({
      R: axAngToMat(uni()*0.2, uni()*0.2, uni()*0.2),
      t: [uni()*0.3, uni()*0.3, uni()*0.3],
    });
  }

  // ── Ground-truth 랜드마크 (카메라 앞쪽 z ∈ [2, 3.5])
  const landmarks_gt = [];
  for (let i = 0; i < numLandmarks; ++i) {
    landmarks_gt.push([uni()*1.0, uni()*1.0, 2 + r()*1.5]);
  }

  // ── 모든 (pose, landmark) 쌍에서 관측 생성 (z > 0.5일 때만)
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

  // ── 초기치: pose 0은 GT 그대로, pose 1+는 GT ± 0.15, landmark는 GT ± 0.2
  const init_poses = [];
  for (let p = 0; p < numPoses; ++p) {
    const ax = matToAxAng(poses_gt[p].R);
    const pert = (p === 0) ? 0 : 0.15;  // fixed pose는 섭동 없음
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

// =============================================================================
// §D. 한 케이스 실행 + 판정
// =============================================================================

/**
 * §5의 두 조건 — chi² 수렴 + 기하 오차 — 을 모두 검사한다.
 *
 * 회전 오차는 각 pose의 $R_{err} = R_{est} R_{gt}^\top$의 axis-angle 크기:
 *
 * $$ \text{rotErr}_p = \| \log(R_{est,p} R_{gt,p}^\top)^{\vee} \| $$
 *
 * 변환 오차는 유클리드 거리:
 *
 * $$ \text{trErr}_p = \| t_{est,p} - t_{gt,p} \|_2, \qquad
 *    \text{lmErr}_l = \| P_w^{est,l} - P_w^{gt,l} \|_2 $$
 *
 * `max` over 모든 pose / landmark를 취한 뒤 허용 한계와 비교.
 */
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

  // 모든 pose / landmark의 최대 오차 집계
  let maxRot = 0, maxTr = 0, maxLm = 0;
  for (let p = 0; p < numPoses; ++p) {
    const M12 = refinedPoses.slice(12*p, 12*p + 12);  // row-major 3×4
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

  // §5(a) chi² 조건: noiseless는 machine-zero, noisy는 3자릿수 감소
  const chi2Pass = noiseStdPx === 0
    ? res.final_chi2 < 1e-10
    : res.final_chi2 < res.initial_chi2 * 1e-3;
  // §5(b) geometric 조건
  const geomPass = maxRot < tolRot && maxTr < tolTr && maxLm < tolLm;
  const pass = chi2Pass && geomPass;

  console.log(
    `[${pass ? 'PASS' : 'FAIL'}] ${label}: obs=${numObs} iters=${res.iterations} ` +
    `chi2 ${res.initial_chi2.toExponential(2)} → ${res.final_chi2.toExponential(2)}  ` +
    `maxRot=${maxRot.toExponential(2)} maxTr=${maxTr.toExponential(2)} maxLm=${maxLm.toExponential(2)}` +
    (pass ? '' : `  ← chi2Pass=${chi2Pass} geomPass=${geomPass}`)
  );
  return pass;
}

// =============================================================================
// §E. 테스트 드라이버 — §6의 매트릭스를 순서대로 실행
// =============================================================================

let all = true;

console.log('[Stage 2] 노이즈 없음 — 기대: chi² → 0 (machine precision)');
console.log('  (monocular scale gauge 때문에 절대 기하 오차는 0이 되지 않음)');
console.log('  허용 한계: chi²_final < 1e-10,  maxRot < 5e-3,  maxTr < 5e-2,  maxLm < 1e-1');
for (let s = 1; s <= 3; ++s) {
  all &= runCase(`  seed=${s} P=3 L=20`, {
    seed: s, numPoses: 3, numLandmarks: 20, noiseStdPx: 0,
    tolRot: 5e-3, tolTr: 5e-2, tolLm: 1e-1,
  });
}

console.log('\n[Stage 2] 1 px Gaussian 노이즈 — 기대: chi² 3+자릿수 감소');
console.log('  허용 한계: chi²_final < chi²_initial × 1e-3,  maxRot < 5e-2,  maxTr < 5e-2,  maxLm < 2e-1');
for (let s = 10; s <= 12; ++s) {
  all &= runCase(`  seed=${s} P=4 L=40`, {
    seed: s, numPoses: 4, numLandmarks: 40, noiseStdPx: 1,
    tolRot: 5e-2, tolTr: 5e-2, tolLm: 2e-1,
  });
}

if (!all) {
  console.error('\n❌ FAIL — Phase A+ Stage 2 gate 미통과.');
  console.error('   §7의 디버깅 가이드 순서대로 의심하라:');
  console.error('     1) Landmark Jacobian (_jacobianOplusXj) 곱셈 순서');
  console.error('     2) setMarginalized(true) 누락 여부');
  console.error('     3) LinearSolverEigen의 실제 sparse 백엔드 확인');
  console.error('     4) RNG seed 전파 / makeScene 구성');
  process.exit(1);
}
console.log('\n✅ OK: g2o BA spike (solver_eigen / SimplicialLLT) converges.');
console.log('   → Phase A+ Stage 2 gate 통과 — Phase G Step 11 BA 경로 확보.');
