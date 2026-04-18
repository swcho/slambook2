// =============================================================================
// verify_pnp.mjs — Phase A+ Stage 1 Gate Test
//
// 이 스크립트는 `bind_pnp_spike.cpp`가 컴파일한 `myslam_pnp_spike.baseline.wasm`
// 이 **g2o 기반 pose-only PnP**를 Emscripten/WASM 환경에서 올바르게 수행하는지
// **수치적으로 증명**한다. Gate 통과 여부는 단 한 줄 — `node verify_pnp.mjs`의
// exit code가 0이면 PASS, 1이면 FAIL.
//
// 자세한 수학적 배경은 ./README.md 참조. 아래는 "본 파일이 주장하는 사양"을
// 자기 완결적으로 기술한다.
//
// -----------------------------------------------------------------------------
// 1. 문제 정의
// -----------------------------------------------------------------------------
// 알려진 3D 점 집합 $\{P_w^i\}_{i=1}^N$ ($P_w^i \in \mathbb{R}^3$)과 그에 대응
// 하는 2D 관측 $\{z^i = (u^i, v^i)\}$이 있고, 내부 파라미터 $K$도 알려져 있다.
// 미지수는 월드 → 카메라 변환 $T_{cw} \in SE(3)$ 하나. 다음 비선형 최소제곱을
// 풀어 $T_{cw}$를 찾는다:
//
//   $$ T_{cw}^* = \arg\min_{T} \sum_{i=1}^N \|z^i - \pi(K\,T\,P_w^i)\|^2 $$
//
// 여기서 $\pi$는 동차 좌표의 정규화: $\pi([\tilde u, \tilde v, \tilde w]^\top)
// = [\tilde u / \tilde w,\; \tilde v / \tilde w]^\top$.
//
// -----------------------------------------------------------------------------
// 2. 합성 데이터 생성 규약
// -----------------------------------------------------------------------------
// Deterministic RNG(Mulberry32) + 시드로 재현 가능한 장면을 만든다:
//   - Ground-truth 포즈 $T_{cw}^{gt}$: 회전 $|\phi| \le 0.3$ rad 이내의 임의
//     axis-angle, translation $t \in [-0.2, 0.2]^2 \times [2.0, 2.5]$.
//   - $N$개의 랜드마크: $P_w \sim \mathcal{U}([-1.5, 1.5]^3)$, $P_c.z < 0.5$인
//     점은 재샘플 (카메라 뒤 또는 너무 가까운 점 제외).
//   - 각 점을 정확히 $K T_{cw}^{gt} P_w$로 투영, 필요시 $\mathcal{N}(0,\sigma^2)$
//     Gaussian 픽셀 노이즈 추가.
//
// -----------------------------------------------------------------------------
// 3. 초기치 규약 — "SLAM-style prior"
// -----------------------------------------------------------------------------
// 실제 ch13 Frontend는 PnP를 **Identity로부터 풀지 않고**, 직전 프레임의 포즈에
// 상대 운동을 합성한 $T_0 = T_{rel} \cdot T_{last}$를 초기치로 쓴다. 이게 없으면
// 먼 점들의 Jacobian linearization이 깨져 LM이 거의 움직이지 못한다 (자세한
// 실험 기록: verify_pnp_diag.mjs / verify_pnp_diag2.mjs).
//
// 본 테스트는 이 패턴을 다음처럼 재현한다:
//   - Init: $T_{cw}^{gt}$에 translation $\pm 0.3$, axis-angle $\pm 0.1$ rad의
//     균일 섭동 추가.
//
// -----------------------------------------------------------------------------
// 4. 테스트 매트릭스
// -----------------------------------------------------------------------------
// (A) **Noiseless**: 5개 시드, N=40, $\sigma = 0$.
//     기대값: 수렴 후 회전·변환 오차가 **machine precision** 근처.
//     허용 한계: rotErr $< 10^{-8}$ rad, trErr $< 10^{-8}$.
//
// (B) **1 px Gaussian noise**: 3개 시드, N=80, $\sigma = 1$ px.
//     기대값: 회전 오차는 mrad 단위, 변환 오차는 cm 단위로 낮아짐.
//     허용 한계: rotErr $< 10^{-2}$ rad, trErr $< 5 \times 10^{-2}$.
//
// Gate는 **모든 케이스가 허용 한계 내에서 통과**할 때만 PASS.
//
// -----------------------------------------------------------------------------
// 5. 통과 기준이 무너진다면 무엇을 의심할지
// -----------------------------------------------------------------------------
//  - chi²가 발산 / LM이 움직이지 않음  → Jacobian 부호 오류, computeError의
//                                         분모 부호, 또는 oplusImpl의 곱 순서
//                                         (left vs right multiplication).
//  - 노이즈 없는 케이스만 실패         → scale gauge / rank deficiency 의심.
//                                         (pose-only PnP는 3D 점이 동일평면이면
//                                          유일해가 없음)
//  - 노이즈 있는 케이스만 실패         → linear solver 수치 안정성, LM λ 초기값.
// =============================================================================

import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const __dirname = dirname(fileURLToPath(import.meta.url));
const wasmDir = resolve(__dirname, '../../public/wasm');
const { default: createModule } = await import(resolve(wasmDir, 'myslam_pnp_spike.baseline.js'));
const M = await createModule({ wasmBinary: readFileSync(resolve(wasmDir, 'myslam_pnp_spike.baseline.wasm')) });

// Embind `VectorDouble` → 평범한 JS 배열 변환 (`.slice()` 미지원).
const toArray = v => { const a = []; for (let i = 0; i < v.size(); ++i) a.push(v.get(i)); return a; };

// =============================================================================
// §A. 3D 수학 도우미 — 모두 row-major 표현을 사용한다
// =============================================================================

/**
 * Rodrigues 공식으로 axis-angle $\phi = \theta \hat{k}$를 회전 행렬로 변환.
 *
 * $$ R = I + \sin\theta\, \hat{k}^{\wedge} + (1 - \cos\theta)\, (\hat{k}^{\wedge})^2 $$
 *
 * $\theta \to 0$ 극한에서는 $R = I$로 fallback (normalize에서 0 나눗셈 회피).
 */
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

/**
 * 회전 행렬 → axis-angle $\phi$. 오차 판정에 사용.
 *
 * $$ \theta = \arccos\left(\frac{\mathrm{tr}(R) - 1}{2}\right),\quad
 *    \hat{k} = \frac{1}{2 \sin\theta}
 *     \begin{bmatrix} R_{32} - R_{23} \\ R_{13} - R_{31} \\ R_{21} - R_{12} \end{bmatrix} $$
 *
 * $\theta \to 0$이면 $\phi = 0$으로 반환. $|\phi|$가 그대로 rotation error로 쓰임.
 */
function matToAxisAngle(R) {
  const trace = R[0] + R[4] + R[8];
  const cosTheta = Math.max(-1, Math.min(1, (trace - 1) / 2));
  const theta = Math.acos(cosTheta);
  if (Math.abs(theta) < 1e-12) return [0, 0, 0];
  const k = theta / (2 * Math.sin(theta));
  return [(R[7]-R[5])*k, (R[2]-R[6])*k, (R[3]-R[1])*k];
}

/** row-major 3×3 곱 $A \cdot B$. */
function rMul(A, B) {
  const r = new Array(9).fill(0);
  for (let i = 0; i < 3; ++i)
    for (let j = 0; j < 3; ++j)
      for (let k = 0; k < 3; ++k) r[i*3+j] += A[i*3+k] * B[k*3+j];
  return r;
}

/** row-major 3×3 전치 $A^\top$. */
function rT(A) { return [A[0],A[3],A[6], A[1],A[4],A[7], A[2],A[5],A[8]]; }

// =============================================================================
// §B. 재현 가능한 난수 + Gaussian 샘플러
// =============================================================================

/**
 * Mulberry32 PRNG — 고정 시드로 재현 가능한 균일 난수열 `[0, 1)` 생성.
 * 테스트 결과가 플랫폼/Node 버전과 독립이 되도록 `Math.random()` 대신 사용한다.
 */
function rng(seed) {
  let a = seed >>> 0;
  return () => {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/**
 * Box–Muller 변환: 균일분포 $U_1, U_2 \sim \mathcal{U}(0, 1)$에서 표준정규 샘플.
 *
 * $$ Z = \sqrt{-2 \ln U_1}\, \cos(2\pi U_2) \sim \mathcal{N}(0, 1) $$
 */
function gauss(rand) {
  const u = Math.max(rand(), 1e-12), v = rand();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

// =============================================================================
// §C. 합성 PnP 케이스 생성
// =============================================================================

/**
 * 하나의 PnP 케이스를 만든다. §2, §3의 규약을 구현한다.
 *
 * 반환:
 *   K             — 9개 float, row-major 3×3
 *   pts3, pts2    — 3N / 2N flat 배열
 *   R_gt, t_gt    — ground-truth 회전·변환 (오차 측정 기준)
 *   init_pose6    — solver에 넘길 초기치 $[t_x, t_y, t_z, r_x, r_y, r_z]$
 *                   (앞 3개는 translation, 뒤 3개는 axis-angle rotation)
 */
function makeCase(seed, N, noiseStdPx) {
  const r = rng(seed);
  const uni = () => r() * 2 - 1;   // [-1, 1) uniform

  // GT 포즈 — 회전은 modest, translation은 카메라 앞 ~2m
  const R_gt = axisAngleToMat(uni() * 0.3, uni() * 0.3, uni() * 0.3);
  const t_gt = [uni() * 0.2, uni() * 0.2, 2 + r() * 0.5];

  const fx = 520, fy = 520, cx = 320, cy = 240;
  const K = [fx, 0, cx, 0, fy, cy, 0, 0, 1];

  const pts3 = [];
  const pts2 = [];
  for (let i = 0; i < N; ++i) {
    const Pw = [uni() * 1.5, uni() * 1.5, uni() * 1.5];
    // P_c = R_gt * P_w + t_gt
    const Pc = [
      R_gt[0]*Pw[0] + R_gt[1]*Pw[1] + R_gt[2]*Pw[2] + t_gt[0],
      R_gt[3]*Pw[0] + R_gt[4]*Pw[1] + R_gt[5]*Pw[2] + t_gt[1],
      R_gt[6]*Pw[0] + R_gt[7]*Pw[1] + R_gt[8]*Pw[2] + t_gt[2],
    ];
    if (Pc[2] <= 0.5) { --i; continue; }  // 카메라 뒤 또는 근접점 재샘플

    // 2D 투영 + Gaussian 픽셀 노이즈 (noiseStdPx = 0이면 noiseless)
    const nu = noiseStdPx ? gauss(r) * noiseStdPx : 0;
    const nv = noiseStdPx ? gauss(r) * noiseStdPx : 0;
    pts3.push(...Pw);
    pts2.push(fx * Pc[0] / Pc[2] + cx + nu, fy * Pc[1] / Pc[2] + cy + nv);
  }

  // SLAM-style prior: GT ± [0.3, 0.3, 0.3, 0.1, 0.1, 0.1]
  const axang_gt = matToAxisAngle(R_gt);
  const init_pose6 = [
    t_gt[0] + uni() * 0.3, t_gt[1] + uni() * 0.3, t_gt[2] + uni() * 0.3,
    axang_gt[0] + uni() * 0.1, axang_gt[1] + uni() * 0.1, axang_gt[2] + uni() * 0.1,
  ];
  return { K, pts3, pts2, R_gt, t_gt, init_pose6 };
}

// =============================================================================
// §D. 한 케이스 실행 + 판정
// =============================================================================

/**
 * 합성 케이스 하나를 풀어서 결과와 GT의 오차를 낸다.
 *
 * 회전 오차는 $R_{err} = R_{est} R_{gt}^\top$의 axis-angle 크기 $|\phi|$로 정의:
 *
 * $$ \text{rotErr} = \| \log(R_{est} R_{gt}^\top)^{\vee} \| = \theta_{err} $$
 *
 * 변환 오차는 유클리드 거리:
 *
 * $$ \text{trErr} = \| t_{est} - t_{gt} \|_2 $$
 *
 * 두 값 모두 허용 한계 이내일 때 PASS.
 */
function runCase(label, opts) {
  const { K, pts3, pts2, R_gt, t_gt, init_pose6 } = makeCase(opts.seed, opts.N, opts.noiseStdPx);
  const res = M.solvePnP({ points3d_flat: pts3, points2d_flat: pts2, K_row_major: K, init_pose6, max_iters: 20 });
  const T = toArray(res.Tcw_row_major);        // row-major 4×4
  const R_est = [T[0],T[1],T[2], T[4],T[5],T[6], T[8],T[9],T[10]];
  const t_est = [T[3], T[7], T[11]];
  const rotErr = Math.hypot(...matToAxisAngle(rMul(R_est, rT(R_gt))));
  const trErr = Math.hypot(t_est[0]-t_gt[0], t_est[1]-t_gt[1], t_est[2]-t_gt[2]);
  const pass = rotErr < opts.tolR && trErr < opts.tolT;
  console.log(
    `[${pass ? 'PASS' : 'FAIL'}] ${label}: iters=${res.iterations} ` +
    `chi2=${res.final_chi2.toExponential(2)} rotErr=${rotErr.toExponential(2)}rad ` +
    `trErr=${trErr.toExponential(2)} (tol rotR=${opts.tolR.toExponential(0)} tolT=${opts.tolT.toExponential(0)})`
  );
  return pass;
}

// =============================================================================
// §E. 테스트 드라이버 — §4의 매트릭스를 순서대로 실행
// =============================================================================

let all = true;

console.log('[Stage 1] 노이즈 없음 — 기대: machine precision까지 수렴');
console.log('  허용 한계: rotErr < 1e-8 rad, trErr < 1e-8');
for (let s = 1; s <= 5; ++s) {
  all &= runCase(`  seed=${s} N=40`, { seed: s, N: 40, noiseStdPx: 0, tolR: 1e-8, tolT: 1e-8 });
}

console.log('\n[Stage 1] 1 px Gaussian 노이즈 — 기대: mrad / cm 단위 오차');
console.log('  허용 한계: rotErr < 1e-2 rad, trErr < 5e-2');
for (let s = 10; s <= 12; ++s) {
  all &= runCase(`  seed=${s} N=80`, { seed: s, N: 80, noiseStdPx: 1, tolR: 1e-2, tolT: 5e-2 });
}

if (!all) {
  console.error('\n❌ FAIL — Phase A+ Stage 1 gate 미통과.');
  console.error('   Jacobian / oplusImpl / computeError 부호 또는 linearization을 먼저 의심하라.');
  console.error('   디버깅 참고: verify_pnp_diag.mjs (computeError 격리 테스트),');
  console.error('              verify_pnp_diag2.mjs (perturbation sweep).');
  process.exit(1);
}
console.log('\n✅ OK: g2o PnP spike가 SLAM-style prior에서 허용 한계 내로 수렴.');
console.log('   → Phase A+ Stage 1 gate 통과.');
