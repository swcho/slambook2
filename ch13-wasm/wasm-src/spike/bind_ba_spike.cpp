// Phase A+ Stage 2 spike: sparse bundle adjustment via g2o + solver_eigen.
//
// Validates that g2o's binary-edge BA pipeline (VertexPose + VertexXYZ +
// EdgeProjection with setMarginalized(true) Schur complement) links and
// converges under Emscripten using Eigen's SimplicialLLT sparse solver. This
// substitutes for CXSparse in the Phase A+ gate — the PLAN.md pivot question
// is g2o-vs-minimal-LM, and any g2o-shipped sparse solver answers it.
// CXSparse-specific friction remains an open question for Phase G.

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <vector>

namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;

class VertexPoseSE3 : public g2o::BaseVertex<6, Eigen::Isometry3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setToOriginImpl() override { _estimate = Eigen::Isometry3d::Identity(); }

  void oplusImpl(const double* update) override {
    Eigen::Map<const Vector6d> xi(update);
    Eigen::Vector3d rho = xi.head<3>();
    Eigen::Vector3d phi = xi.tail<3>();
    Eigen::AngleAxisd aa(phi.norm(), phi.norm() > 1e-12 ? phi.normalized() : Eigen::Vector3d::UnitZ());
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

// Binary edge: pose + landmark, 2D reprojection residual.
class EdgeProjection
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPoseSE3, VertexXYZ> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit EdgeProjection(const Eigen::Matrix3d& K) : K_(K) {}

  void computeError() override {
    const auto* vp = static_cast<const VertexPoseSE3*>(_vertices[0]);
    const auto* vx = static_cast<const VertexXYZ*>(_vertices[1]);
    Eigen::Vector3d Pc = vp->estimate() * vx->estimate();
    Eigen::Vector3d uvw = K_ * Pc;
    _error = _measurement - Eigen::Vector2d(uvw.x() / uvw.z(), uvw.y() / uvw.z());
  }

  void linearizeOplus() override {
    const auto* vp = static_cast<const VertexPoseSE3*>(_vertices[0]);
    const auto* vx = static_cast<const VertexXYZ*>(_vertices[1]);
    const Eigen::Isometry3d& T = vp->estimate();
    Eigen::Vector3d Pc = T * vx->estimate();
    const double X = Pc.x(), Y = Pc.y(), Z = Pc.z();
    const double Z2 = Z * Z;
    const double fx = K_(0, 0), fy = K_(1, 1);

    // Pose Jacobian (2x6), [de/drho | de/dphi] — matches ch13 g2o_types.h.
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

    // Landmark Jacobian (2x3) — de/dPw = de/dPc * T.R (no cam_ext; single camera).
    _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) * T.linear();
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }

 private:
  Eigen::Matrix3d K_;
};

// -------- BA input / output --------------------------------------------
struct BAInput {
  // Flat [tx,ty,tz,rx,ry,rz,tx,ty,tz,rx,ry,rz,...] — length = 6 * num_poses.
  emscripten::val init_poses;
  // Flat [x,y,z, x,y,z, ...] — length = 3 * num_landmarks.
  emscripten::val init_landmarks;
  // Flat [pose_idx, landmark_idx, u, v, ...] — each observation is 4 entries.
  emscripten::val observations;
  // 9 entries, row-major K.
  emscripten::val K_row_major;
  // Pose id(s) to fix (usually [0]).
  emscripten::val fixed_poses;
  int max_iters;
};

struct BAResult {
  std::vector<double> refined_poses;       // 12 * num_poses (row-major 3x4 per pose)
  std::vector<double> refined_landmarks;   // 3 * num_landmarks
  int iterations;
  double initial_chi2;
  double final_chi2;
};

static std::vector<double> to_vec(const emscripten::val& v) {
  const unsigned len = v["length"].as<unsigned>();
  std::vector<double> out(len);
  for (unsigned i = 0; i < len; ++i) out[i] = v[i].as<double>();
  return out;
}
static std::vector<int> to_ivec(const emscripten::val& v) {
  const unsigned len = v["length"].as<unsigned>();
  std::vector<int> out(len);
  for (unsigned i = 0; i < len; ++i) out[i] = v[i].as<int>();
  return out;
}

static Eigen::Isometry3d pose6ToIsometry(const double* p6) {
  Eigen::Vector3d t(p6[0], p6[1], p6[2]);
  Eigen::Vector3d r(p6[3], p6[4], p6[5]);
  Eigen::AngleAxisd aa(r.norm(), r.norm() > 1e-12 ? r.normalized() : Eigen::Vector3d::UnitZ());
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = aa.toRotationMatrix();
  T.translation() = t;
  return T;
}

BAResult solveBA(const BAInput& in) {
  std::vector<double> poses_in = to_vec(in.init_poses);
  std::vector<double> lms_in = to_vec(in.init_landmarks);
  std::vector<double> obs = to_vec(in.observations);
  std::vector<double> Kflat = to_vec(in.K_row_major);
  std::vector<int> fixed = to_ivec(in.fixed_poses);

  const int num_poses = static_cast<int>(poses_in.size() / 6);
  const int num_lms = static_cast<int>(lms_in.size() / 3);
  const int num_obs = static_cast<int>(obs.size() / 4);

  Eigen::Matrix3d K;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) K(r, c) = Kflat[r * 3 + c];

  // Solver: block <6,3> with Eigen sparse (SimplicialLLT).
  using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
  using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

  auto linear = std::make_unique<LinearSolverType>();
  auto block = std::make_unique<BlockSolverType>(std::move(linear));
  auto algo = new g2o::OptimizationAlgorithmLevenberg(std::move(block));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(algo);
  optimizer.setVerbose(false);

  std::vector<VertexPoseSE3*> pose_verts(num_poses);
  for (int i = 0; i < num_poses; ++i) {
    auto* v = new VertexPoseSE3();
    v->setId(i);
    v->setEstimate(pose6ToIsometry(&poses_in[6 * i]));
    for (int f : fixed) {
      if (f == i) v->setFixed(true);
    }
    optimizer.addVertex(v);
    pose_verts[i] = v;
  }

  std::vector<VertexXYZ*> lm_verts(num_lms);
  for (int i = 0; i < num_lms; ++i) {
    auto* v = new VertexXYZ();
    v->setId(num_poses + i);
    v->setEstimate(Eigen::Vector3d(lms_in[3 * i], lms_in[3 * i + 1], lms_in[3 * i + 2]));
    v->setMarginalized(true);  // Schur complement — pose block first.
    optimizer.addVertex(v);
    lm_verts[i] = v;
  }

  for (int o = 0; o < num_obs; ++o) {
    const int pi = static_cast<int>(obs[4 * o]);
    const int li = static_cast<int>(obs[4 * o + 1]);
    const double u = obs[4 * o + 2];
    const double v = obs[4 * o + 3];
    auto* e = new EdgeProjection(K);
    e->setId(o);
    e->setVertex(0, pose_verts[pi]);
    e->setVertex(1, lm_verts[li]);
    e->setMeasurement(Eigen::Vector2d(u, v));
    e->setInformation(Eigen::Matrix2d::Identity());
    optimizer.addEdge(e);
  }

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  const double chi0 = optimizer.activeChi2();
  const int iters = optimizer.optimize(in.max_iters > 0 ? in.max_iters : 15);
  const double chi1 = optimizer.activeChi2();

  BAResult r;
  r.refined_poses.resize(12 * num_poses);
  for (int i = 0; i < num_poses; ++i) {
    Eigen::Matrix<double, 3, 4> M;
    M.leftCols<3>() = pose_verts[i]->estimate().linear();
    M.rightCols<1>() = pose_verts[i]->estimate().translation();
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 4; ++b) r.refined_poses[12 * i + 4 * a + b] = M(a, b);
  }
  r.refined_landmarks.resize(3 * num_lms);
  for (int i = 0; i < num_lms; ++i) {
    r.refined_landmarks[3 * i + 0] = lm_verts[i]->estimate().x();
    r.refined_landmarks[3 * i + 1] = lm_verts[i]->estimate().y();
    r.refined_landmarks[3 * i + 2] = lm_verts[i]->estimate().z();
  }
  r.iterations = iters;
  r.initial_chi2 = chi0;
  r.final_chi2 = chi1;
  return r;
}

}  // namespace

EMSCRIPTEN_BINDINGS(ba_spike) {
  emscripten::value_object<BAInput>("BAInput")
      .field("init_poses", &BAInput::init_poses)
      .field("init_landmarks", &BAInput::init_landmarks)
      .field("observations", &BAInput::observations)
      .field("K_row_major", &BAInput::K_row_major)
      .field("fixed_poses", &BAInput::fixed_poses)
      .field("max_iters", &BAInput::max_iters);

  emscripten::value_object<BAResult>("BAResult")
      .field("refined_poses", &BAResult::refined_poses)
      .field("refined_landmarks", &BAResult::refined_landmarks)
      .field("iterations", &BAResult::iterations)
      .field("initial_chi2", &BAResult::initial_chi2)
      .field("final_chi2", &BAResult::final_chi2);

  emscripten::register_vector<double>("VectorDouble");

  emscripten::function("solveBA", &solveBA);
}
