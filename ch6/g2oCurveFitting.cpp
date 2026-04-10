#include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

using namespace std;

// 곡선 모델의 정점, 템플릿 파라미터: 최적화 변수 차원 및 데이터 타입
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 초기화
  virtual void setToOriginImpl() override {
    _estimate << 0, 0, 0;
  }

  // 업데이트
  virtual void oplusImpl(const double *update) override {
    _estimate += Eigen::Vector3d(update);
  }

  // 파일 저장 및 읽기: 비워 둠
  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}
};

// 오차 모델, 템플릿 파라미터: 관측값 차원, 타입, 연결된 정점 타입
class CurveFittingEdge : public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CurveFittingEdge(double x) : BaseUnaryEdge(), _x(x) {}

  // 곡선 모델 오차 계산
  virtual void computeError() override {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    _error(0, 0) = _measurement - std::exp(abc(0, 0) * _x * _x + abc(1, 0) * _x + abc(2, 0));
  }

  // 야코비 행렬 계산
  virtual void linearizeOplus() override {
    const CurveFittingVertex *v = static_cast<const CurveFittingVertex *> (_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    double y = exp(abc[0] * _x * _x + abc[1] * _x + abc[2]);
    _jacobianOplusXi[0] = -_x * _x * y;
    _jacobianOplusXi[1] = -_x * y;
    _jacobianOplusXi[2] = -y;
  }

  virtual bool read(istream &in) {}

  virtual bool write(ostream &out) const {}

public:
  double _x;  // x 값, y 값은 _measurement
};

int main(int argc, char **argv) {
  double ar = 1.0, br = 2.0, cr = 1.0;         // 실제 파라미터 값
  double ae = 2.0, be = -1.0, ce = 5.0;        // 추정 파라미터 값
  int N = 100;                                 // 데이터 포인트 수
  double w_sigma = 1.0;                        // 노이즈 Sigma 값
  double inv_sigma = 1.0 / w_sigma;
  cv::RNG rng;                                 // OpenCV 난수 생성기

  vector<double> x_data, y_data;      // 데이터
  for (int i = 0; i < N; i++) {
    double x = i / 100.0;
    x_data.push_back(x);
    y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
  }

  // 그래프 최적화를 구성합니다. 먼저 g2o 를 설정합니다
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>> BlockSolverType;  // 각 오차 항의 최적화 변수 차원 3, 오차 차원 1
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // 선형 솔버 타입

  // 경사 하강법 선택: GN, LM, DogLeg 중에서 선택 가능
  auto solver = new g2o::OptimizationAlgorithmGaussNewton(
    g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;     // 그래프 모델
  optimizer.setAlgorithm(solver);   // 솔버 설정
  optimizer.setVerbose(true);       // 디버그 출력 활성화

  // 그래프에 정점을 추가합니다
  CurveFittingVertex *v = new CurveFittingVertex();
  v->setEstimate(Eigen::Vector3d(ae, be, ce));
  v->setId(0);
  optimizer.addVertex(v);

  // 그래프에 엣지를 추가합니다
  for (int i = 0; i < N; i++) {
    CurveFittingEdge *edge = new CurveFittingEdge(x_data[i]);
    edge->setId(i);
    edge->setVertex(0, v);                // 연결된 정점을 설정합니다
    edge->setMeasurement(y_data[i]);      // 관측값
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * 1 / (w_sigma * w_sigma)); // 정보 행렬: 공분산 행렬의 역행렬
    optimizer.addEdge(edge);
  }

  // 최적화 실행
  cout << "start optimization" << endl;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  // 최적화 결과 출력
  Eigen::Vector3d abc_estimate = v->estimate();
  cout << "estimated model: " << abc_estimate.transpose() << endl;

  return 0;
}
