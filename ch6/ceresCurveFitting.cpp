//
// Created by xiang on 18-11-19.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

// 비용 함수 계산 모델
struct CURVE_FITTING_COST {
  CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

  // 잔차 계산
  template<typename T>
  bool operator()(const T *const abc, // 모델 파라미터, 3차원
                  T *residual) const {
    residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]); // y-exp(ax^2+bx+c)
    return true;
  }

  const double _x, _y;    // x, y 데이터
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

  double abc[3] = {ae, be, ce};

  // 최소제곱 문제를 구성합니다
  ceres::Problem problem;
  for (int i = 0; i < N; i++) {
    problem.AddResidualBlock(     // 문제에 잔차 항을 추가합니다
      // 자동 미분 사용, 템플릿 파라미터: 오차 타입, 출력 차원, 입력 차원 (앞의 struct 와 일치해야 합니다)
      new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
        new CURVE_FITTING_COST(x_data[i], y_data[i])
      ),
      nullptr,            // 커널 함수, 여기서는 사용하지 않아 nullptr
      abc                 // 추정할 파라미터
    );
  }

  // 솔버 설정
  ceres::Solver::Options options;     // 다양한 설정 항목이 있습니다
  options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 증분 방정식 풀이 방법
  options.minimizer_progress_to_stdout = true;   // cout 으로 출력

  ceres::Solver::Summary summary;                // 최적화 정보
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);  // 최적화 시작
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  // 결과 출력
  cout << summary.BriefReport() << endl;
  cout << "estimated a,b,c = ";
  for (auto a:abc) cout << a << " ";
  cout << endl;

  return 0;
}
