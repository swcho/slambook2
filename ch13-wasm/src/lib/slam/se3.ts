// Minimal SE(3) helpers for the Phase H Map manager (Step 12 + Step 13).
//
// We cannot depend on Sophus or g2o for these operations on the JS side, so
// this file provides just enough to mirror the few ch13 call sites we need:
//   - SE3 composition  (kf.pose * Twc)
//   - SE3 inversion    (frame->Pose().inverse() == Twc)
//   - SE3 log-norm     ((kf.pose * Twc).log().norm())  ← duplicate KF detector
//
// All matrices are row-major Float64Array. We use length-16 (Mat4) for general
// SE(3) and length-12 (Mat3x4) for poses-without-bottom-row, matching the
// layout already used by ba.ts.

export type Mat4 = Float64Array; // length 16, row-major
export type Mat3x4 = Float64Array; // length 12, row-major
export type Vec3 = [number, number, number];

export function mat4Identity(): Mat4 {
  return new Float64Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
}

export function mat4FromMat3x4(m12: Mat3x4): Mat4 {
  if (m12.length !== 12) throw new Error(`Mat3x4 expects 12, got ${m12.length}`);
  const out = new Float64Array(16);
  out.set(m12, 0);
  out[12] = 0;
  out[13] = 0;
  out[14] = 0;
  out[15] = 1;
  return out;
}

export function mat3x4FromMat4(m16: Mat4): Mat3x4 {
  if (m16.length !== 16) throw new Error(`Mat4 expects 16, got ${m16.length}`);
  return new Float64Array(m16.buffer.slice(m16.byteOffset, m16.byteOffset + 12 * 8));
}

/** Multiply two row-major 4×4 matrices: out = a · b. */
export function mat4Multiply(a: Mat4, b: Mat4): Mat4 {
  const out = new Float64Array(16);
  for (let r = 0; r < 4; r++) {
    for (let c = 0; c < 4; c++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += a[r * 4 + k] * b[k * 4 + c];
      out[r * 4 + c] = s;
    }
  }
  return out;
}

/** Invert an SE(3) [R|t] (row-major 4×4): [R^T | -R^T t]. */
export function se3Invert(m: Mat4): Mat4 {
  const r00 = m[0], r01 = m[1], r02 = m[2];
  const r10 = m[4], r11 = m[5], r12 = m[6];
  const r20 = m[8], r21 = m[9], r22 = m[10];
  const tx = m[3], ty = m[7], tz = m[11];
  // R^T t
  const itx = -(r00 * tx + r10 * ty + r20 * tz);
  const ity = -(r01 * tx + r11 * ty + r21 * tz);
  const itz = -(r02 * tx + r12 * ty + r22 * tz);
  return new Float64Array([
    r00, r10, r20, itx,
    r01, r11, r21, ity,
    r02, r12, r22, itz,
    0, 0, 0, 1,
  ]);
}

/** Translation of an SE(3): the last column of R|t. */
export function se3Translation(m: Mat4): Vec3 {
  return [m[3], m[7], m[11]];
}

/**
 * Compute ‖log(T)‖ for T ∈ SE(3) — the metric ch13 `RemoveOldKeyframe` uses
 * to score "how close is this KF to the current one?"
 *
 * log: SE3 → se3, T = (R, t) → ξ = (ρ, φ) ∈ ℝ⁶  with
 *   φ = log(R)               (Rodrigues axis-angle vector)
 *   ρ = J⁻¹(φ) · t            (left-Jacobian inverse from sophus)
 * Returns √(‖ρ‖² + ‖φ‖²).
 */
export function se3LogNorm(m: Mat4): number {
  const r00 = m[0], r01 = m[1], r02 = m[2];
  const r10 = m[4], r11 = m[5], r12 = m[6];
  const r20 = m[8], r21 = m[9], r22 = m[10];
  const tx = m[3], ty = m[7], tz = m[11];

  // φ = log(R) via Rodrigues angle θ.
  const tr = r00 + r11 + r22;
  const cosTheta = Math.max(-1, Math.min(1, (tr - 1) / 2));
  const theta = Math.acos(cosTheta);

  let phi: Vec3;
  if (theta < 1e-8) {
    // sin θ ~ θ; the axis is the antisymmetric part / 2.
    phi = [
      0.5 * (r21 - r12),
      0.5 * (r02 - r20),
      0.5 * (r10 - r01),
    ];
  } else {
    const k = theta / (2 * Math.sin(theta));
    phi = [k * (r21 - r12), k * (r02 - r20), k * (r10 - r01)];
  }
  const phiNorm2 = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];

  // J⁻¹(φ) — closed-form left-Jacobian inverse (Sophus impl).
  // For small θ this collapses to identity; here we use the general form to
  // keep accuracy on duplicate-cluster boundary cases.
  let rho: Vec3;
  if (theta < 1e-8) {
    rho = [tx, ty, tz];
  } else {
    // J⁻¹ = I - 0.5 [φ]× + ((1 - θ cot(θ/2) / 2) / θ²) [φ]×²
    const halfCot = (theta * 0.5) / Math.tan(theta * 0.5);
    const c2 = (1 - halfCot) / phiNorm2;
    const rho0 = applyJacobianInverse(phi, [tx, ty, tz], c2);
    rho = rho0;
  }
  const rhoNorm2 = rho[0] * rho[0] + rho[1] * rho[1] + rho[2] * rho[2];
  return Math.sqrt(rhoNorm2 + phiNorm2);
}

function applyJacobianInverse(phi: Vec3, t: Vec3, c2: number): Vec3 {
  // J⁻¹ t = t - 0.5 (φ × t) + c2 (φ × (φ × t))
  const cross1: Vec3 = [
    phi[1] * t[2] - phi[2] * t[1],
    phi[2] * t[0] - phi[0] * t[2],
    phi[0] * t[1] - phi[1] * t[0],
  ];
  const cross2: Vec3 = [
    phi[1] * cross1[2] - phi[2] * cross1[1],
    phi[2] * cross1[0] - phi[0] * cross1[2],
    phi[0] * cross1[1] - phi[1] * cross1[0],
  ];
  return [
    t[0] - 0.5 * cross1[0] + c2 * cross2[0],
    t[1] - 0.5 * cross1[1] + c2 * cross2[1],
    t[2] - 0.5 * cross1[2] + c2 * cross2[2],
  ];
}

/** Build a pure-translation SE(3). */
export function se3FromTranslation(t: Vec3): Mat4 {
  return new Float64Array([
    1, 0, 0, t[0],
    0, 1, 0, t[1],
    0, 0, 1, t[2],
    0, 0, 0, 1,
  ]);
}

/** Build an SE(3) with rotation around y-axis (pitch in radians) and translation. */
export function se3FromYawTranslation(yawRad: number, t: Vec3): Mat4 {
  const c = Math.cos(yawRad);
  const s = Math.sin(yawRad);
  return new Float64Array([
    c, 0, s, t[0],
    0, 1, 0, t[1],
    -s, 0, c, t[2],
    0, 0, 0, 1,
  ]);
}
