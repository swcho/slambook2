// Phase D / Step 5 — DLT triangulation (Linear SVD) + Midpoint with σ4/σ3
// quality metric. Mirrors ch13/include/myslam/algorithm.h::triangulation but
// exposes the quality ratio to JS so the UI can chart per-point conditioning
// and drive the "inverted-return" book-bug toggle (PLAN §3 Step 5).
//
// Embind API:
//   triangulate(leftPts, rightPts, kL[4], tL[12], kR[4], tR[12], opts) →
//     Float64Array stride 5 [x, y, z, qualityMetric, ok]
//
//   - leftPts / rightPts: stride-3 Float64Array [x, y, score|status, ...]
//     (matches Step 3 detectFeatures / Step 4 trackLK output). When the third
//     element ≤ 0.5 the point is treated as a missing match (ok = 0, NaN
//     metric) — same convention as bind_features.trackLK.
//   - kL, kR: [fx, fy, cx, cy] (4 doubles each).
//   - tL, tR: 3×4 row-major world→camera transform [R(3×3) | t(3)] as 12
//     doubles. For KITTI rectified stereo pass R = I and t = (0,0,0) for the
//     left camera, t = (-baseline, 0, 0) for the right camera.
//   - opts.algo:                 0 = LinearSVD (default), 1 = Midpoint.
//   - opts.qualityThreshold:     σ4/σ3 cutoff (default 0.01, matches ch13).
//                                 For Midpoint, interpreted as
//                                 ray-distance ÷ mean-depth.
//   - opts.invertedReturn (bool, default false): when true, accept the
//     points that fail the quality check (literal interpretation of
//     algorithm.h's "포기" comment). PLAN §3 Step 5 학습 포인트.

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <cmath>
#include <cstddef>
#include <limits>

namespace myslam {

namespace {

enum AlgorithmKind : int {
  kLinearSVD = 0,
  kMidpoint = 1,
};

Eigen::Matrix<double, 3, 4> readT(const emscripten::val& src) {
  Eigen::Matrix<double, 3, 4> M;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 4; ++c) {
      M(r, c) = src[r * 4 + c].as<double>();
    }
  }
  return M;
}

}  // namespace

emscripten::val triangulateBatch(
    emscripten::val leftPts, emscripten::val rightPts,
    emscripten::val kL, emscripten::val tL,
    emscripten::val kR, emscripten::val tR,
    emscripten::val opts) {
  emscripten::val Float64Array = emscripten::val::global("Float64Array");
  const std::size_t lLen = leftPts["length"].as<std::size_t>();
  const std::size_t rLen = rightPts["length"].as<std::size_t>();
  if (lLen != rLen || lLen % 3 != 0) return Float64Array.new_(0);
  const std::size_t n = lLen / 3;

  const double fxL = kL[0u].as<double>();
  const double fyL = kL[1u].as<double>();
  const double cxL = kL[2u].as<double>();
  const double cyL = kL[3u].as<double>();
  const double fxR = kR[0u].as<double>();
  const double fyR = kR[1u].as<double>();
  const double cxR = kR[2u].as<double>();
  const double cyR = kR[3u].as<double>();

  const Eigen::Matrix<double, 3, 4> Ml = readT(tL);  // world → left
  const Eigen::Matrix<double, 3, 4> Mr = readT(tR);  // world → right

  auto getInt = [&](const char* key, int def) {
    if (opts.isUndefined() || opts.isNull()) return def;
    emscripten::val v = opts[key];
    return v.isUndefined() ? def : v.as<int>();
  };
  auto getNum = [&](const char* key, double def) {
    if (opts.isUndefined() || opts.isNull()) return def;
    emscripten::val v = opts[key];
    return v.isUndefined() ? def : v.as<double>();
  };
  auto getBool = [&](const char* key, bool def) {
    if (opts.isUndefined() || opts.isNull()) return def;
    emscripten::val v = opts[key];
    return v.isUndefined() ? def : v.as<bool>();
  };
  const int algo = getInt("algo", static_cast<int>(kLinearSVD));
  const double qualityThreshold = getNum("qualityThreshold", 0.01);
  const bool invertedReturn = getBool("invertedReturn", false);

  const Eigen::Matrix3d Rl = Ml.block<3, 3>(0, 0);
  const Eigen::Matrix3d Rr = Mr.block<3, 3>(0, 0);
  const Eigen::Vector3d tl = Ml.col(3);
  const Eigen::Vector3d tr = Mr.col(3);
  // Camera centres in world frame: solve R*C + t = 0 → C = -R^T t.
  const Eigen::Vector3d Cl = -Rl.transpose() * tl;
  const Eigen::Vector3d Cr = -Rr.transpose() * tr;

  emscripten::val out = Float64Array.new_(n * 5);
  for (std::size_t i = 0; i < n; ++i) {
    const double uL = leftPts[i * 3 + 0].as<double>();
    const double vL = leftPts[i * 3 + 1].as<double>();
    const double uR = rightPts[i * 3 + 0].as<double>();
    const double vR = rightPts[i * 3 + 1].as<double>();
    const double status = rightPts[i * 3 + 2].as<double>();

    if (status <= 0.5) {
      out.set(i * 5 + 0, 0.0);
      out.set(i * 5 + 1, 0.0);
      out.set(i * 5 + 2, 0.0);
      out.set(i * 5 + 3, std::numeric_limits<double>::quiet_NaN());
      out.set(i * 5 + 4, 0.0);
      continue;
    }

    // Normalised image-plane points (K⁻¹ · [u v 1]).
    const Eigen::Vector3d pnL((uL - cxL) / fxL, (vL - cyL) / fyL, 1.0);
    const Eigen::Vector3d pnR((uR - cxR) / fxR, (vR - cyR) / fyR, 1.0);

    Eigen::Vector3d X = Eigen::Vector3d::Zero();
    double metric = 0.0;
    bool qualityOk = false;

    if (algo == kMidpoint) {
      // Ray directions in world: d = R^T · pn.
      Eigen::Vector3d dl = Rl.transpose() * pnL;
      Eigen::Vector3d dr = Rr.transpose() * pnR;
      const double dlNorm = dl.norm();
      const double drNorm = dr.norm();
      if (dlNorm > 0) dl /= dlNorm;
      if (drNorm > 0) dr /= drNorm;
      Eigen::Matrix2d AA;
      AA(0, 0) = dl.dot(dl);
      AA(0, 1) = -dl.dot(dr);
      AA(1, 0) = -dl.dot(dr);
      AA(1, 1) = dr.dot(dr);
      const Eigen::Vector3d w = Cr - Cl;
      Eigen::Vector2d bb;
      bb(0) = dl.dot(w);
      bb(1) = -dr.dot(w);
      const Eigen::Vector2d st = AA.fullPivLu().solve(bb);
      const Eigen::Vector3d PlPt = Cl + st(0) * dl;
      const Eigen::Vector3d PrPt = Cr + st(1) * dr;
      X = 0.5 * (PlPt + PrPt);
      const double rayDist = (PlPt - PrPt).norm();
      const double depthL = (Rl * X + tl).norm();
      const double depthR = (Rr * X + tr).norm();
      const double meanDepth = std::max(1e-6, 0.5 * (depthL + depthR));
      metric = rayDist / meanDepth;
      qualityOk = metric < qualityThreshold;
    } else {
      // Linear SVD (DLT) — mirrors ch13 algorithm.h.
      Eigen::Matrix4d A;
      A.row(0) = pnL(0) * Ml.row(2) - Ml.row(0);
      A.row(1) = pnL(1) * Ml.row(2) - Ml.row(1);
      A.row(2) = pnR(0) * Mr.row(2) - Mr.row(0);
      A.row(3) = pnR(1) * Mr.row(2) - Mr.row(1);
      Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
      const Eigen::Vector4d xh = svd.matrixV().col(3);
      const double w = xh(3);
      if (std::abs(w) > 0) {
        X = xh.head<3>() / w;
      } else {
        X = xh.head<3>();
      }
      const auto& sv = svd.singularValues();
      metric = sv(2) > 0 ? sv(3) / sv(2)
                          : std::numeric_limits<double>::infinity();
      qualityOk = metric < qualityThreshold;
    }

    // Depth in left-camera frame (z-component of R_l X + t_l).
    const double depth = (Rl * X + tl).z();
    const bool depthOk = depth > 0;
    const bool ok = invertedReturn ? (!qualityOk && depthOk)
                                    : (qualityOk && depthOk);

    out.set(i * 5 + 0, X(0));
    out.set(i * 5 + 1, X(1));
    out.set(i * 5 + 2, X(2));
    out.set(i * 5 + 3, metric);
    out.set(i * 5 + 4, ok ? 1.0 : 0.0);
  }
  return out;
}

}  // namespace myslam

EMSCRIPTEN_BINDINGS(myslam_triangulation) {
  emscripten::function("triangulate", &myslam::triangulateBatch);
  emscripten::constant("ALGO_LINEAR_SVD", 0);
  emscripten::constant("ALGO_MIDPOINT", 1);
}
