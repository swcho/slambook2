#include <iostream>

using namespace std;

#include <ctime>
// Eigen 핵심 모듈
#include <Eigen/Core>
// 밀집 행렬의 대수 연산 (역행렬, 고유값 등)
#include <Eigen/Dense>

using namespace Eigen;

#define MATRIX_SIZE 50

/****************************
* 이 프로그램은 Eigen 기본 타입의 사용법을 보여줍니다
****************************/

int main(int argc, char **argv) {
  // Eigen의 모든 벡터와 행렬은 Eigen::Matrix 템플릿 클래스입니다. 앞 세 파라미터는 데이터 타입, 행, 열입니다
  // 2x3 float 행렬 선언
  Matrix<float, 2, 3> matrix_23;

  // Eigen은 typedef로 다양한 내장 타입을 제공하지만, 내부적으로는 모두 Eigen::Matrix입니다
  // 예를 들어 Vector3d는 실제로 Eigen::Matrix<double, 3, 1>, 즉 3차원 벡터입니다
  Vector3d v_3d;
  // 아래도 동일합니다
  Matrix<float, 3, 1> vd_3d;

  // Matrix3d는 실제로 Eigen::Matrix<double, 3, 3>입니다
  Matrix3d matrix_33 = Matrix3d::Zero(); // 0으로 초기화
  // 행렬 크기가 불확실한 경우 동적 크기 행렬을 사용할 수 있습니다
  Matrix<double, Dynamic, Dynamic> matrix_dynamic;
  // 더 간단하게
  MatrixXd matrix_x;
  // 이 외에도 많은 타입이 있지만 일일이 나열하지 않습니다

  // 아래는 Eigen 행렬 연산입니다
  // 데이터 입력 (초기화)
  matrix_23 << 1, 2, 3, 4, 5, 6;
  // 출력
  cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << endl;

  // ()로 행렬 원소에 접근
  cout << "print matrix 2x3: " << endl;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 3; j++) cout << matrix_23(i, j) << "\t";
    cout << endl;
  }

  // 행렬과 벡터의 곱셈 (실제로는 행렬과 행렬의 곱)
  v_3d << 3, 2, 1;
  vd_3d << 4, 5, 6;

  // Eigen에서는 서로 다른 타입의 행렬을 혼합할 수 없습니다. 아래처럼 하면 오류가 납니다
  // Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
  // 명시적으로 타입 변환해야 합니다
  Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
  cout << "[1,2,3;4,5,6]*[3,2,1]=" << result.transpose() << endl;

  Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
  cout << "[1,2,3;4,5,6]*[4,5,6]: " << result2.transpose() << endl;

  // 마찬가지로 행렬의 차원도 틀리면 안 됩니다
  // 아래 주석을 해제하면 Eigen이 어떤 오류를 발생시키는지 확인해 보세요
  // Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

  // 몇 가지 행렬 연산
  // 사칙연산은 생략합니다. +-*/를 그대로 사용하면 됩니다
  matrix_33 = Matrix3d::Random();      // 난수 행렬
  cout << "random matrix: \n" << matrix_33 << endl;
  cout << "transpose: \n" << matrix_33.transpose() << endl;      // 전치
  cout << "sum: " << matrix_33.sum() << endl;            // 원소 합
  cout << "trace: " << matrix_33.trace() << endl;          // 대각합 (trace)
  cout << "times 10: \n" << 10 * matrix_33 << endl;               // 스칼라 곱
  cout << "inverse: \n" << matrix_33.inverse() << endl;        // 역행렬
  cout << "det: " << matrix_33.determinant() << endl;    // 행렬식

  // 고유값
  // 실수 대칭 행렬은 대각화가 보장됩니다
  SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
  cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
  cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;

  // 방정식 풀기
  // matrix_NN * x = v_Nd 를 풉니다
  // N의 크기는 앞의 매크로에서 정의되며, 난수로 생성됩니다
  // 직접 역행렬을 구하는 것이 가장 직관적이지만, 연산량이 많습니다

  Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN
      = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
  matrix_NN = matrix_NN * matrix_NN.transpose();  // 반양정치 행렬 보장
  Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

  clock_t time_stt = clock(); // 시간 측정 시작
  // 직접 역행렬로 풀기
  Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
  cout << "time of normal inverse is "
       << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
  cout << "x = " << x.transpose() << endl;

  // 보통 행렬 분해로 풀면 훨씬 빠릅니다. 예: QR 분해
  time_stt = clock();
  x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
  cout << "time of Qr decomposition is "
       << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
  cout << "x = " << x.transpose() << endl;

  // 양정치 행렬의 경우 Cholesky 분해(LDLT)로도 풀 수 있습니다
  time_stt = clock();
  x = matrix_NN.ldlt().solve(v_Nd);
  cout << "time of ldlt decomposition is "
       << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
  cout << "x = " << x.transpose() << endl;

  return 0;
}
