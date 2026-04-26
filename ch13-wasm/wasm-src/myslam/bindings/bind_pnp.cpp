// Phase E / Step 8 — Pose-only PnP (g2o + Eigen) production binding.
//
// Promotes wasm-src/spike/bind_pnp_spike.cpp into the real binding while
// faithfully mirroring ch13/src/frontend.cpp::EstimateCurrentPose:
//   - 4 rounds × 10 iterations (configurable)
//   - chi² > 5.991 ⇒ outlier ⇒ edge.setLevel(1) for the next round
//   - drops RobustKernel after a configurable round (book uses round ≥ 2)
//
// API additions over the spike (kept in Float64Array stride-3 form to match
// the rest of ch13-wasm: features/triangulation):
//   estimatePose(points3dFlat, obs2dFlat, kRowMajor, initPose6, opts)
//     → { Tcw_row_major, finalInlierMask, roundInlierMasks (R*N),
//         roundChi2Sum, roundInlierCount, totalIterations, finalChi2 }
//
// The roundInlierMasks/roundChi2Sum tables are what the Step 8 UI renders as
// the "각 라운드별 inlier 변화 애니메이션" called out in PLAN §3 Step 8.

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace myslam {
namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;

// Same minimal SE(3) left-update vertex as the Phase A+ spike. Kept verbatim
// (modulo namespace) so the gate's wire-level behavior is preserved.
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

class EdgeReprojectionPoseOnly
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPoseSE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeReprojectionPoseOnly(const Eigen::Vector3d& point_world,
                           const Eigen::Matrix3d& K)
      : point_world_(point_world), K_(K) {}

  void computeError() override {
    const auto* v = static_cast<const VertexPoseSE3*>(_vertices[0]);
    const Eigen::Isometry3d& Tcw = v->estimate();
    Eigen::Vector3d Pc = Tcw * point_world_;
    Eigen::Vector3d uvw = K_ * Pc;
    Eigen::Vector2d uv(uvw.x() / uvw.z(), uvw.y() / uvw.z());
    _error = _measurement - uv;
  }

  void linearizeOplus() override {
    const auto* v = static_cast<const VertexPoseSE3*>(_vertices[0]);
    const Eigen::Isometry3d& Tcw = v->estimate();
    Eigen::Vector3d Pc = Tcw * point_world_;
    const double X = Pc.x(), Y = Pc.y(), Z = Pc.z();
    const double Z2 = Z * Z;
    const double fx = K_(0, 0), fy = K_(1, 1);

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
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }

 private:
  Eigen::Vector3d point_world_;
  Eigen::Matrix3d K_;
};

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

emscripten::val packI32(const std::vector<int>& v) {
  emscripten::val Int32Array = emscripten::val::global("Int32Array");
  emscripten::val out = Int32Array.new_(v.size());
  for (std::size_t i = 0; i < v.size(); ++i) out.set(i, v[i]);
  return out;
}

// estimatePose mirrors ch13/src/frontend.cpp::EstimateCurrentPose:
//   * graph = 1 VertexPoseSE3 + N EdgeReprojectionPoseOnly
//   * each edge gets a Huber RobustKernel with δ = sqrt(chi2Threshold)
//   * `rounds` × `iterPerRound` LM iterations
//   * after each round, edges with chi² > chi2Threshold are flagged outliers
//     (setLevel(1)), and starting from `removeKernelAfterRound` the kernel
//     is detached so the optimizer chases the inliers to higher precision.
//
// Inputs (all flat — Float64Array):
//   points3dFlat   — 3*N world points       [x, y, z, ...]
//   obs2dFlat      — 2*N observed pixels    [u, v, ...]
//   kRowMajor      — 9 entries              [fx, 0, cx, 0, fy, cy, 0, 0, 1]
//   initPose6      — 6 entries (optional)   [tx, ty, tz, rx, ry, rz] (axis-angle)
//                    Pass length-0 array for identity init.
emscripten::val estimatePose(emscripten::val points3dFlat,
                             emscripten::val obs2dFlat,
                             emscripten::val kRowMajor,
                             emscripten::val initPose6,
                             emscripten::val opts) {
  std::vector<double> pts3 = toDouble(points3dFlat);
  std::vector<double> pts2 = toDouble(obs2dFlat);
  std::vector<double> kFlat = toDouble(kRowMajor);

  if (pts3.size() % 3 != 0 || pts2.size() % 2 != 0 || kFlat.size() != 9) {
    return emscripten::val::object();
  }
  const int N = static_cast<int>(pts3.size() / 3);
  if (static_cast<int>(pts2.size() / 2) != N) {
    return emscripten::val::object();
  }

  Eigen::Matrix3d K;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) K(r, c) = kFlat[r * 3 + c];

  Eigen::Isometry3d T0 = Eigen::Isometry3d::Identity();
  if (initPose6["length"].as<unsigned>() == 6) {
    std::vector<double> p6 = toDouble(initPose6);
    Eigen::Vector3d t(p6[0], p6[1], p6[2]);
    Eigen::Vector3d r(p6[3], p6[4], p6[5]);
    Eigen::AngleAxisd aa(
        r.norm(),
        r.norm() > 1e-12 ? r.normalized() : Eigen::Vector3d::UnitZ());
    T0.linear() = aa.toRotationMatrix();
    T0.translation() = t;
  }

  // Defaults match ch13 frontend.cpp::EstimateCurrentPose verbatim.
  const int rounds = std::max(1, optInt<int>(opts, "rounds", 4));
  const int iterPerRound = std::max(1, optInt<int>(opts, "iterPerRound", 10));
  const double chi2Th = optNum(opts, "chi2Threshold", 5.991);
  // Book uses δ = sqrt(5.991). Expose so learners can dial Huber.
  const double huberDelta =
      optNum(opts, "huberDelta", std::sqrt(std::max(chi2Th, 1e-12)));
  // ch13 drops the kernel for the last two rounds (i.e. starting at round 2
  // when counting from 0). Allow learners to push it later or earlier.
  const int dropKernelAt =
      optInt<int>(opts, "removeKernelAfterRound", 2);
  const bool useRobustKernel = optBool(opts, "useRobustKernel", true);

  using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
  using LinearSolverType =
      g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

  auto linear = std::make_unique<LinearSolverType>();
  auto block = std::make_unique<BlockSolverType>(std::move(linear));
  auto algo = new g2o::OptimizationAlgorithmLevenberg(std::move(block));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(algo);
  optimizer.setVerbose(false);

  auto* vpose = new VertexPoseSE3();
  vpose->setId(0);
  vpose->setEstimate(T0);
  optimizer.addVertex(vpose);

  std::vector<EdgeReprojectionPoseOnly*> edges(N);
  std::vector<std::uint8_t> outlier(N, 0);  // sticky across rounds
  for (int i = 0; i < N; ++i) {
    Eigen::Vector3d Pw(pts3[3 * i], pts3[3 * i + 1], pts3[3 * i + 2]);
    Eigen::Vector2d uv(pts2[2 * i], pts2[2 * i + 1]);
    auto* e = new EdgeReprojectionPoseOnly(Pw, K);
    e->setId(i);
    e->setVertex(0, vpose);
    e->setMeasurement(uv);
    e->setInformation(Eigen::Matrix2d::Identity());
    if (useRobustKernel) {
      auto* rk = new g2o::RobustKernelHuber();
      rk->setDelta(huberDelta);
      e->setRobustKernel(rk);
    }
    optimizer.addEdge(e);
    edges[i] = e;
  }

  // Per-round outputs, flattened:
  //   roundMasks  : R * N (1=inlier this round, 0=outlier)
  //   roundChi2   : R * N (per-edge chi² after this round's optimize step)
  std::vector<std::uint8_t> roundMasks(rounds * N, 0);
  std::vector<double> roundChi2(rounds * N, 0.0);
  std::vector<int> roundInlierCount(rounds, 0);
  std::vector<double> roundChi2Sum(rounds, 0.0);
  std::vector<int> roundIters(rounds, 0);

  for (int round = 0; round < rounds; ++round) {
    // Re-arm or strip the kernel before optimize, like frontend.cpp does
    // inside its main for-loop.
    for (int i = 0; i < N; ++i) {
      auto* e = edges[i];
      e->setLevel(outlier[i] ? 1 : 0);
      if (useRobustKernel && round < dropKernelAt) {
        if (e->robustKernel() == nullptr) {
          auto* rk = new g2o::RobustKernelHuber();
          rk->setDelta(huberDelta);
          e->setRobustKernel(rk);
        }
      } else {
        // ch13: `e->setRobustKernel(nullptr)` for round >= dropKernelAt.
        e->setRobustKernel(nullptr);
      }
    }

    optimizer.initializeOptimization(0);
    roundIters[round] = optimizer.optimize(iterPerRound);

    // Re-classify after this round's optimize. Mirrors ch13's per-round
    // inlier counting: edges currently marked outlier still get computeError
    // run so we surface their chi² for the UI animation.
    int nInlier = 0;
    double chiSum = 0.0;
    for (int i = 0; i < N; ++i) {
      auto* e = edges[i];
      e->computeError();
      const double chi2 = e->chi2();
      const std::size_t idx = static_cast<std::size_t>(round) * N + i;
      roundChi2[idx] = chi2;
      const bool isOutlier = chi2 > chi2Th;
      outlier[i] = isOutlier ? 1 : 0;
      roundMasks[idx] = isOutlier ? 0 : 1;
      if (!isOutlier) {
        ++nInlier;
        chiSum += chi2;
      }
    }
    roundInlierCount[round] = nInlier;
    roundChi2Sum[round] = chiSum;
  }

  // Final pose + final inlier mask snapshot.
  const Eigen::Isometry3d Tcw = vpose->estimate();
  std::vector<double> Tcw_row_major(16);
  Eigen::Matrix4d M = Tcw.matrix();
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) Tcw_row_major[r * 4 + c] = M(r, c);

  std::vector<std::uint8_t> finalInlierMask(N);
  int totalInliers = 0;
  for (int i = 0; i < N; ++i) {
    finalInlierMask[i] = outlier[i] ? 0 : 1;
    if (!outlier[i]) ++totalInliers;
  }

  emscripten::val res = emscripten::val::object();
  res.set("Tcw_row_major", packF64(Tcw_row_major));
  res.set("finalInlierMask", packU8(finalInlierMask));
  res.set("roundInlierMasks", packU8(roundMasks));
  res.set("roundChi2Values", packF64(roundChi2));
  res.set("roundChi2Sum", packF64(roundChi2Sum));
  res.set("roundInlierCount", packI32(roundInlierCount));
  res.set("roundIters", packI32(roundIters));
  res.set("totalInliers", totalInliers);
  res.set("finalChi2", optimizer.activeChi2());
  res.set("rounds", rounds);
  res.set("N", N);
  return res;
}

}  // namespace
}  // namespace myslam

EMSCRIPTEN_BINDINGS(myslam_pnp) {
  emscripten::function("estimatePose", &myslam::estimatePose);
}
