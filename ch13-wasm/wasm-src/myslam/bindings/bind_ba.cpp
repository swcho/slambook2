// Phase G / Step 11 — Bundle Adjustment (g2o + Eigen sparse) production binding.
//
// Promotes wasm-src/spike/bind_ba_spike.cpp into the real binding while
// faithfully mirroring ch13/src/backend.cpp::Optimize:
//   - Binary edges (pose, landmark) with per-edge left/right camera extrinsic
//     (book uses one EdgeProjection per stereo branch).
//   - Schur complement via setMarginalized(true) on landmark vertices.
//   - LM solve for `iterations` LM steps, then an adaptive chi² loop that
//     doubles chi²_th up to `adaptiveRounds` times until inlier ratio > 0.5
//     (same as backend.cpp's `while(iteration < 5)`).
//   - Per-edge initial / final chi² + final inlier mask exposed to the UI.
//
// Inputs (all flat — Float64Array):
//   initPoses12Flat       — 12 * P (row-major 3×4 per pose; T_cw)
//   initLandmarks3Flat    —  3 * L (world-frame XYZ per landmark)
//   observationsFlat      —  5 * O (poseIdx, lmIdx, u, v, isLeftImage)
//   fixedPoseIndicesFlat  — list of pose ids to fix (usually [0])
//   kRowMajor             — 9 entries [fx, 0, cx, 0, fy, cy, 0, 0, 1]
//   leftExt12 / rightExt12— 12 each, row-major 3×4 left/right camera extrinsic
//
// Outputs:
//   refinedPoses12        — 12 * P (row-major 3×4)
//   refinedLandmarks3     —  3 * L
//   perEdgeChi2Initial    —  O — chi² before optimization (with current init)
//   perEdgeChi2Final      —  O — chi² after the full adaptive chi² loop
//   finalInlierMask       —  O — 1=inlier (chi² ≤ finalChi2Threshold), 0=outlier
//   initialChi2Sum        — sum over all observations
//   finalChi2Sum          — sum over all observations
//   iterations            — number of LM iterations actually consumed
//   finalChi2Threshold    — chi² threshold after adaptive doubling
//   adaptiveDoublings     — how many times the threshold doubled
//   finalInlierRatio      — count(inliers) / O at the end
//
// The Step 11 UI uses (perEdgeChi2Initial, perEdgeChi2Final, finalInlierMask)
// to render the "전/후 reprojection error 히스토그램 + bar chart" called out in
// PLAN §3 Step 11.

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace myslam {
namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;

// SE(3) left-update vertex. Identical math to bind_pnp.cpp::VertexPoseSE3
// and the spike, kept verbatim so all three binaries share wire-level behavior.
class VertexPoseSE3 : public g2o::BaseVertex<6, Eigen::Isometry3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setToOriginImpl() override { _estimate = Eigen::Isometry3d::Identity(); }

  void oplusImpl(const double* update) override {
    Eigen::Map<const Vector6d> xi(update);
    Eigen::Vector3d rho = xi.head<3>();
    Eigen::Vector3d phi = xi.tail<3>();
    Eigen::AngleAxisd aa(
        phi.norm(),
        phi.norm() > 1e-12 ? phi.normalized() : Eigen::Vector3d::UnitZ());
    Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
    dT.linear() = aa.toRotationMatrix();
    dT.translation() = rho;
    _estimate = dT * _estimate;
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};

class VertexXYZ : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setToOriginImpl() override { _estimate.setZero(); }

  void oplusImpl(const double* update) override {
    _estimate += Eigen::Map<const Eigen::Vector3d>(update);
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};

// Binary edge: pose + landmark with a per-camera extrinsic.
// computeError mirrors ch13 backend.cpp's `_K * (_cam_ext * (T * Pw))` chain.
class EdgeProjection
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPoseSE3, VertexXYZ> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeProjection(const Eigen::Matrix3d& K, const Eigen::Isometry3d& cam_ext)
      : K_(K), cam_ext_(cam_ext) {}

  void computeError() override {
    const auto* vp = static_cast<const VertexPoseSE3*>(_vertices[0]);
    const auto* vx = static_cast<const VertexXYZ*>(_vertices[1]);
    Eigen::Vector3d Pc = cam_ext_ * (vp->estimate() * vx->estimate());
    Eigen::Vector3d uvw = K_ * Pc;
    _error = _measurement - Eigen::Vector2d(uvw.x() / uvw.z(), uvw.y() / uvw.z());
  }

  void linearizeOplus() override {
    const auto* vp = static_cast<const VertexPoseSE3*>(_vertices[0]);
    const auto* vx = static_cast<const VertexXYZ*>(_vertices[1]);
    const Eigen::Isometry3d& T = vp->estimate();
    Eigen::Vector3d Pc = cam_ext_ * (T * vx->estimate());
    const double X = Pc.x(), Y = Pc.y(), Z = Pc.z();
    const double Z2 = Z * Z;
    const double fx = K_(0, 0), fy = K_(1, 1);

    // Pose Jacobian (2x6) — matches ch13 g2o_types.h::EdgeProjection.
    _jacobianOplusXi(0, 0) = -fx / Z;
    _jacobianOplusXi(0, 1) = 0;
    _jacobianOplusXi(0, 2) = fx * X / Z2;
    _jacobianOplusXi(0, 3) = fx * X * Y / Z2;
    _jacobianOplusXi(0, 4) = -fx - fx * X * X / Z2;
    _jacobianOplusXi(0, 5) = fx * Y / Z;
    _jacobianOplusXi(1, 0) = 0;
    _jacobianOplusXi(1, 1) = -fy / Z;
    _jacobianOplusXi(1, 2) = fy * Y / Z2;
    _jacobianOplusXi(1, 3) = fy + fy * Y * Y / Z2;
    _jacobianOplusXi(1, 4) = -fy * X * Y / Z2;
    _jacobianOplusXi(1, 5) = -fy * X / Z;

    // Landmark Jacobian (2x3) — de/dPw = de/dPc * (cam_ext.R * T.R).
    // Same chain rule as ch13 g2o_types.h::EdgeProjection.
    _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                       cam_ext_.linear() * T.linear();
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }

 private:
  Eigen::Matrix3d K_;
  Eigen::Isometry3d cam_ext_;
};

// ─── tiny helpers ──────────────────────────────────────────────────────────
std::vector<double> toDouble(const emscripten::val& v) {
  const unsigned len = v["length"].as<unsigned>();
  std::vector<double> out(len);
  for (unsigned i = 0; i < len; ++i) out[i] = v[i].as<double>();
  return out;
}

template <typename T>
T optInt(const emscripten::val& opts, const char* key, T def) {
  if (opts.isUndefined() || opts.isNull()) return def;
  emscripten::val v = opts[key];
  return v.isUndefined() ? def : static_cast<T>(v.as<int>());
}

double optNum(const emscripten::val& opts, const char* key, double def) {
  if (opts.isUndefined() || opts.isNull()) return def;
  emscripten::val v = opts[key];
  return v.isUndefined() ? def : v.as<double>();
}

bool optBool(const emscripten::val& opts, const char* key, bool def) {
  if (opts.isUndefined() || opts.isNull()) return def;
  emscripten::val v = opts[key];
  return v.isUndefined() ? def : v.as<bool>();
}

emscripten::val packF64(const std::vector<double>& v) {
  emscripten::val Float64Array = emscripten::val::global("Float64Array");
  emscripten::val out = Float64Array.new_(v.size());
  for (std::size_t i = 0; i < v.size(); ++i) out.set(i, v[i]);
  return out;
}

emscripten::val packU8(const std::vector<std::uint8_t>& v) {
  emscripten::val Uint8Array = emscripten::val::global("Uint8Array");
  emscripten::val out = Uint8Array.new_(v.size());
  for (std::size_t i = 0; i < v.size(); ++i) out.set(i, v[i]);
  return out;
}

// Decode a row-major 3×4 [R|t] block into an Eigen::Isometry3d.
Eigen::Isometry3d decodePose12(const double* p) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  Eigen::Matrix3d R;
  R << p[0], p[1], p[2],
       p[4], p[5], p[6],
       p[8], p[9], p[10];
  T.linear() = R;
  T.translation() = Eigen::Vector3d(p[3], p[7], p[11]);
  return T;
}

void encodePose12(const Eigen::Isometry3d& T, double* out) {
  Eigen::Matrix3d R = T.linear();
  Eigen::Vector3d t = T.translation();
  out[0]  = R(0, 0); out[1]  = R(0, 1); out[2]  = R(0, 2); out[3]  = t.x();
  out[4]  = R(1, 0); out[5]  = R(1, 1); out[6]  = R(1, 2); out[7]  = t.y();
  out[8]  = R(2, 0); out[9]  = R(2, 1); out[10] = R(2, 2); out[11] = t.z();
}

// optimize() runs the full ch13 backend.cpp::Optimize pipeline:
//   1. Build VertexPose × P (with optional fixed) + VertexXYZ × L (marginalized).
//   2. Build EdgeProjection × O picking left or right cam_ext per observation.
//   3. optimize(iterations) once.
//   4. Adaptive chi² loop: while inlier_ratio ≤ adaptiveInlierRatio and we
//      still have rounds left, double chi²_th. (No more LM iterations — the
//      book just relaxes the threshold; the loop terminates when ≥ 50% of
//      edges fall under it.)
//   5. Mark final inliers/outliers, write back refined poses + landmarks.
emscripten::val optimize(emscripten::val initPoses12Flat,
                         emscripten::val initLandmarks3Flat,
                         emscripten::val observationsFlat,
                         emscripten::val fixedPoseIndicesFlat,
                         emscripten::val kRowMajor,
                         emscripten::val leftExt12,
                         emscripten::val rightExt12,
                         emscripten::val opts) {
  std::vector<double> poses_in  = toDouble(initPoses12Flat);
  std::vector<double> lms_in    = toDouble(initLandmarks3Flat);
  std::vector<double> obs       = toDouble(observationsFlat);
  std::vector<double> fixedFlat = toDouble(fixedPoseIndicesFlat);
  std::vector<double> kFlat     = toDouble(kRowMajor);
  std::vector<double> Lflat     = toDouble(leftExt12);
  std::vector<double> Rflat     = toDouble(rightExt12);

  if (poses_in.size() % 12 != 0 || lms_in.size() % 3 != 0 ||
      obs.size() % 5 != 0 || kFlat.size() != 9 ||
      Lflat.size() != 12 || Rflat.size() != 12) {
    return emscripten::val::object();
  }

  const int P = static_cast<int>(poses_in.size() / 12);
  const int L = static_cast<int>(lms_in.size() / 3);
  const int O = static_cast<int>(obs.size() / 5);

  Eigen::Matrix3d K;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) K(r, c) = kFlat[r * 3 + c];

  Eigen::Isometry3d leftExt  = decodePose12(Lflat.data());
  Eigen::Isometry3d rightExt = decodePose12(Rflat.data());

  // Defaults match ch13/src/backend.cpp::Optimize verbatim.
  const int iters = std::max(1, optInt<int>(opts, "iterations", 10));
  const double chi2Init = optNum(opts, "chi2Init", 5.991);
  const int adaptiveRounds = std::max(0, optInt<int>(opts, "adaptiveRounds", 5));
  const double adaptiveInlierRatio =
      optNum(opts, "adaptiveInlierRatio", 0.5);
  const bool useRobustKernel = optBool(opts, "useRobustKernel", true);
  // Book uses Huber δ = chi2_th literally (not sqrt). Keep that default but
  // expose a knob so learners can experiment with the more-conventional
  // δ = sqrt(chi2_th) parameterization.
  const double huberDelta = optNum(opts, "huberDelta", chi2Init);

  using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
  using LinearSolverType =
      g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

  auto linear = std::make_unique<LinearSolverType>();
  auto block = std::make_unique<BlockSolverType>(std::move(linear));
  auto algo = new g2o::OptimizationAlgorithmLevenberg(std::move(block));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(algo);
  optimizer.setVerbose(false);

  // ── Pose vertices.
  std::vector<VertexPoseSE3*> pose_verts(P);
  for (int p = 0; p < P; ++p) {
    auto* v = new VertexPoseSE3();
    v->setId(p);
    v->setEstimate(decodePose12(&poses_in[12 * p]));
    optimizer.addVertex(v);
    pose_verts[p] = v;
  }
  for (double idx : fixedFlat) {
    int i = static_cast<int>(idx);
    if (i >= 0 && i < P) pose_verts[i]->setFixed(true);
  }

  // ── Landmark vertices (Schur complement via setMarginalized(true)).
  std::vector<VertexXYZ*> lm_verts(L);
  for (int l = 0; l < L; ++l) {
    auto* v = new VertexXYZ();
    v->setId(P + l);
    v->setEstimate(Eigen::Vector3d(lms_in[3 * l],
                                   lms_in[3 * l + 1],
                                   lms_in[3 * l + 2]));
    v->setMarginalized(true);
    optimizer.addVertex(v);
    lm_verts[l] = v;
  }

  // ── Edges. Pick left or right cam_ext per observation.
  std::vector<EdgeProjection*> edges(O);
  for (int o = 0; o < O; ++o) {
    const int pi = static_cast<int>(obs[5 * o + 0]);
    const int li = static_cast<int>(obs[5 * o + 1]);
    const double u = obs[5 * o + 2];
    const double v = obs[5 * o + 3];
    const bool isLeft = obs[5 * o + 4] != 0.0;
    auto* e = new EdgeProjection(K, isLeft ? leftExt : rightExt);
    e->setId(o);
    e->setVertex(0, pose_verts[pi]);
    e->setVertex(1, lm_verts[li]);
    e->setMeasurement(Eigen::Vector2d(u, v));
    e->setInformation(Eigen::Matrix2d::Identity());
    if (useRobustKernel) {
      auto* rk = new g2o::RobustKernelHuber();
      rk->setDelta(huberDelta);
      e->setRobustKernel(rk);
    }
    optimizer.addEdge(e);
    edges[o] = e;
  }

  // ── Capture per-edge chi² before optimization (init geometry residual).
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  std::vector<double> perEdgeChi2Initial(O, 0.0);
  for (int o = 0; o < O; ++o) perEdgeChi2Initial[o] = edges[o]->chi2();
  const double initialChi2Sum = optimizer.activeChi2();

  const int actualIters = optimizer.optimize(iters);

  // ── Adaptive chi² loop (same shape as ch13 backend.cpp).
  double chi2_th = chi2Init;
  int adaptiveDoublings = 0;
  for (int round = 0; round < adaptiveRounds; ++round) {
    int nIn = 0, nOut = 0;
    for (int o = 0; o < O; ++o) {
      if (edges[o]->chi2() > chi2_th) ++nOut; else ++nIn;
    }
    const int total = nIn + nOut;
    const double ratio = total > 0 ? static_cast<double>(nIn) / total : 1.0;
    if (ratio > adaptiveInlierRatio) break;
    chi2_th *= 2.0;
    ++adaptiveDoublings;
  }

  // ── Final per-edge classification + chi² snapshot for the UI.
  std::vector<double> perEdgeChi2Final(O, 0.0);
  std::vector<std::uint8_t> finalInlierMask(O, 0);
  int inlierCount = 0;
  double finalChi2Sum = 0.0;
  for (int o = 0; o < O; ++o) {
    const double c = edges[o]->chi2();
    perEdgeChi2Final[o] = c;
    finalChi2Sum += c;
    if (c <= chi2_th) {
      finalInlierMask[o] = 1;
      ++inlierCount;
    }
  }

  // ── Pack refined poses and landmarks.
  std::vector<double> refinedPoses12(12 * P, 0.0);
  for (int p = 0; p < P; ++p) {
    encodePose12(pose_verts[p]->estimate(), &refinedPoses12[12 * p]);
  }
  std::vector<double> refinedLandmarks3(3 * L, 0.0);
  for (int l = 0; l < L; ++l) {
    refinedLandmarks3[3 * l + 0] = lm_verts[l]->estimate().x();
    refinedLandmarks3[3 * l + 1] = lm_verts[l]->estimate().y();
    refinedLandmarks3[3 * l + 2] = lm_verts[l]->estimate().z();
  }

  emscripten::val res = emscripten::val::object();
  res.set("refinedPoses12", packF64(refinedPoses12));
  res.set("refinedLandmarks3", packF64(refinedLandmarks3));
  res.set("perEdgeChi2Initial", packF64(perEdgeChi2Initial));
  res.set("perEdgeChi2Final", packF64(perEdgeChi2Final));
  res.set("finalInlierMask", packU8(finalInlierMask));
  res.set("initialChi2Sum", initialChi2Sum);
  res.set("finalChi2Sum", finalChi2Sum);
  res.set("iterations", actualIters);
  res.set("finalChi2Threshold", chi2_th);
  res.set("adaptiveDoublings", adaptiveDoublings);
  res.set("finalInlierCount", inlierCount);
  res.set("finalInlierRatio", O > 0 ? static_cast<double>(inlierCount) / O : 1.0);
  res.set("P", P);
  res.set("L", L);
  res.set("O", O);
  return res;
}

}  // namespace
}  // namespace myslam

EMSCRIPTEN_BINDINGS(myslam_ba) {
  emscripten::function("optimize", &myslam::optimize);
}
