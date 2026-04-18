// Phase A+ spike: pose-only PnP using g2o + Eigen under Emscripten.
//
// Mirrors ch13/include/myslam/g2o_types.h::EdgeProjectionPoseOnly but uses
// a minimal 6-DOF se(3) manifold (Eigen's AngleAxis/Translation) instead of
// Sophus, to avoid pulling in the Sophus submodule just for the spike.
// Stage 1 validates that g2o core + solvers/dense link and converge against
// synthetic ground-truth projections. Stage 2 (CXSparse + BA) lives
// separately.

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <vector>

namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;

// Minimal SE(3) left-multiplicative update using so(3) + R^3 tangent vector.
// xi = [rho (translation); phi (rotation)] — same convention as Sophus::SE3::log.
class VertexPoseSE3 : public g2o::BaseVertex<6, Eigen::Isometry3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setToOriginImpl() override { _estimate = Eigen::Isometry3d::Identity(); }

  void oplusImpl(const double* update) override {
    Eigen::Map<const Vector6d> xi(update);
    Eigen::Vector3d rho = xi.head<3>();
    Eigen::Vector3d phi = xi.tail<3>();

    // SE(3) left update: T' = exp(xi) * T. We compose with a small increment.
    Eigen::AngleAxisd aa(phi.norm(), phi.norm() > 1e-12 ? phi.normalized() : Eigen::Vector3d::UnitZ());
    Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();
    dT.linear() = aa.toRotationMatrix();
    dT.translation() = rho;
    _estimate = dT * _estimate;
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};

// Unary edge: 2D reprojection error of a fixed 3D landmark under pinhole K.
// residual = observed_px - project(K * (R * Xw + t)).
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

    // Analytic 2x6 Jacobian, matching EdgeProjectionPoseOnly in g2o_types.h.
    // Columns: [drho (3) | dphi (3)] with left-multiplicative update.
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

struct PnPInput {
  // Flat column-major [x0,y0,z0,x1,y1,z1,...]; length = 3*N.
  emscripten::val points3d_flat;
  // Flat [u0,v0,u1,v1,...]; length = 2*N.
  emscripten::val points2d_flat;
  // 9 entries, row-major: fx,0,cx, 0,fy,cy, 0,0,1.
  emscripten::val K_row_major;
  // 6-vec initial pose guess [tx,ty,tz,rx,ry,rz] (axis-angle) — optional.
  emscripten::val init_pose6;
  int max_iters;
};

struct PnPResult {
  // 16 entries, row-major 4x4 Tcw.
  std::vector<double> Tcw_row_major;
  int iterations;
  double final_chi2;
  bool converged;
};

PnPResult solvePnP(const PnPInput& in) {
  auto to_vec = [](const emscripten::val& v) {
    const unsigned len = v["length"].as<unsigned>();
    std::vector<double> out(len);
    for (unsigned i = 0; i < len; ++i) out[i] = v[i].as<double>();
    return out;
  };

  std::vector<double> pts3 = to_vec(in.points3d_flat);
  std::vector<double> pts2 = to_vec(in.points2d_flat);
  std::vector<double> Kflat = to_vec(in.K_row_major);
  const int N = static_cast<int>(pts3.size() / 3);

  Eigen::Matrix3d K;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) K(r, c) = Kflat[r * 3 + c];

  Eigen::Isometry3d T0 = Eigen::Isometry3d::Identity();
  if (in.init_pose6["length"].as<unsigned>() == 6) {
    std::vector<double> p6 = to_vec(in.init_pose6);
    Eigen::Vector3d t(p6[0], p6[1], p6[2]);
    Eigen::Vector3d r(p6[3], p6[4], p6[5]);
    Eigen::AngleAxisd aa(r.norm(), r.norm() > 1e-12 ? r.normalized() : Eigen::Vector3d::UnitZ());
    T0.linear() = aa.toRotationMatrix();
    T0.translation() = t;
  }

  // Solver: 6x6 pose block (PnP is small, dense works fine).
  using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
  using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

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

  for (int i = 0; i < N; ++i) {
    Eigen::Vector3d Pw(pts3[3 * i], pts3[3 * i + 1], pts3[3 * i + 2]);
    Eigen::Vector2d uv(pts2[2 * i], pts2[2 * i + 1]);
    auto* e = new EdgeReprojectionPoseOnly(Pw, K);
    e->setId(i);
    e->setVertex(0, vpose);
    e->setMeasurement(uv);
    e->setInformation(Eigen::Matrix2d::Identity());
    optimizer.addEdge(e);
  }

  optimizer.initializeOptimization();
  const int iters = optimizer.optimize(in.max_iters > 0 ? in.max_iters : 10);

  Eigen::Isometry3d Tcw = vpose->estimate();
  PnPResult result;
  result.Tcw_row_major.resize(16);
  Eigen::Matrix4d M = Tcw.matrix();
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) result.Tcw_row_major[r * 4 + c] = M(r, c);
  result.iterations = iters;
  result.final_chi2 = optimizer.activeChi2();
  result.converged = iters > 0;
  return result;
}

}  // namespace

EMSCRIPTEN_BINDINGS(pnp_spike) {
  emscripten::value_object<PnPInput>("PnPInput")
      .field("points3d_flat", &PnPInput::points3d_flat)
      .field("points2d_flat", &PnPInput::points2d_flat)
      .field("K_row_major", &PnPInput::K_row_major)
      .field("init_pose6", &PnPInput::init_pose6)
      .field("max_iters", &PnPInput::max_iters);

  emscripten::value_object<PnPResult>("PnPResult")
      .field("Tcw_row_major", &PnPResult::Tcw_row_major)
      .field("iterations", &PnPResult::iterations)
      .field("final_chi2", &PnPResult::final_chi2)
      .field("converged", &PnPResult::converged);

  emscripten::register_vector<double>("VectorDouble");

  emscripten::function("solvePnP", &solvePnP);
}
