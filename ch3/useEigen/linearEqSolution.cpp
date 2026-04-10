# include <iostream>
# include <Eigen/Dense>
# include <Eigen/Core>
#include <random> // C++11 난수 라이브러리 사용


using namespace std;
using namespace Eigen;

// 사용자 정의 가우스 소거법 함수
VectorXd gaussianElimination(const MatrixXd& A, const VectorXd& b) {
    const Index n = A.rows(); // 행렬의 행 수 (즉 방정식의 개수)

    // A 와 b 를 합쳐 첨가행렬 [A | b] 로 만들기
    MatrixXd augmented(n, n + 1);
    augmented << A, b;

    // 소거 단계: 행렬 A 를 상삼각행렬로 변환
    for (Index k = 0; k < n; ++k) {
        // 1. 주원소(pivot) 선택: 현재 대각선 원소가 0이 아닌지 확인
        for (Index i = k + 1; i < n; ++i) {
            if (augmented(k, k) == 0) {
                cerr << "Zero pivot encountered. Matrix is singular!" << endl;
                exit(EXIT_FAILURE);
            }
            // 2. k 열의 i 행 원소를 소거
            const double factor = augmented(i, k) / augmented(k, k);
            for (Index j = k; j <= n; ++j) {
                augmented(i, j) -= factor * augmented(k, j);
            }
        }
    }

    // 후진 대입 단계: 마지막 행부터 미지수를 순서대로 풀기
    VectorXd x(n);
    for (Index i = n - 1; i >= 0; --i) {
        x(i) = augmented(i, n);
        for (Index j = i + 1; j < n; ++j) {
            x(i) -= augmented(i, j) * x(j);
        }
        x(i) /= augmented(i, i);
    }

    return x;
}

int main() {
    cout.precision(3); // 限制精确度

    std::random_device rd;                          // 고품질 난수 시드 획득
    std::default_random_engine generator(rd());     // 난수 생성기 초기화
    std::uniform_real_distribution<double> distribution(-1.0, 1.0); // 균등 분포 [-1, 1]

    // Eigen 행렬 초기화
    MatrixXd B1(3, 3); // rows x cols 크기의 행렬 생성

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            B1(i, j) = distribution(generator); // 각 원소를 난수로 채우기
        }
    }

    EigenSolver<Matrix3d> const solver1(B1);
    MatrixXd A1 = B1.transpose() * B1 + 0.1 * MatrixXd::Identity(3, 3);

    const VectorXcd eigenValueA = solver1.eigenvalues();

    cout << "eigenValue of A = \n" << eigenValueA << endl;

  const VectorXd b1 = VectorXd::Random(3,1);

    // 1.1 사용자 정의 가우스 소거법(Gaussian Elimination) 함수 호출
    const VectorXd gex = gaussianElimination(A1, b1);

    // 输出解
    cout << "Solution of Gaussian Elimination: x = \n" << gex << endl;

    // 1.2 使用Eigen解决Ax=b
    const VectorXd ex = A1.partialPivLu().solve(b1);
    cout << "Solution of Eigen: x = \n" << ex << endl;

  // 2. 使用LU分解法求解线性方程Ax=b
  const Matrix<double, 3, 1> lux = A1.lu().solve(b1); // A.lu().solve(b) 的 lu() 分解适用于方阵（n×n）

  cout << "Solution of lu decomposition: x = \n" << lux << endl;

  // 3. 使用LLT分解法求解线性方程Ax=b，要求A是对称正定矩阵
  LLT<Matrix<double, 3, 3>> const llt(A1); // LLT 分解
  if (llt.info() == Success) {
    const Matrix<double, 3, 1> lltx = llt.solve(b1);
    cout << "Solution of LLT decomposition: x = \n" << lltx << endl;
  } else {
    cout << "Matrix A is not positive definite!" << endl;
  }

    // 4. 使用QR分解法
    // QR 分解
    HouseholderQR<MatrixXd> const qr(A1);

    // 求解 Ax = b
    VectorXd qrx = qr.solve(b1);

    // 获取 Q 和 R
    MatrixXd Q = qr.householderQ();
    MatrixXd R = qr.matrixQR().triangularView<Upper>();

    cout << "Solution of QR decomposition: x = \n" << qrx << endl;
    cout << "Q = \n" << Q << "\nR = \n" << R << endl;

    // 5. 奇异值分解 (SVD) 求解
    // 对矩阵 A 进行奇异值分解
    JacobiSVD<MatrixXd> const svd(A1, ComputeThinU | ComputeThinV);

    // 使用 SVD 求解 Ax = b
    const VectorXd svdx = svd.solve(b1);

    cout << "Solution of SVD: x = \n" << svdx << endl;

    // 输出奇异值
    cout << "Singular values of A:\n" << svd.singularValues() << endl;

    // 使用特征值分解求解
    // 使用 EigenSolver 进行特征值分解
    EigenSolver<Matrix3d> solver2(A1);

    // 获取特征向量矩阵 V 和特征值对角矩阵 Lambda
    Matrix3d V = solver2.eigenvectors().real(); // 提取实部特征向量
    Vector3d Lambda = solver2.eigenvalues().real(); // 提取实部特征值

    // 计算中间变量 y = V.inverse() * b
    Vector3d y = V.inverse() * b1;

    // 通过 Lambda 求解 y，并还原 x = V * y
    for (int i = 0; i < 3; ++i) {
        if (Lambda(i) != 0) { // 避免除以 0
            y(i) /= Lambda(i);
        } else {
            cerr << "Error: Zero eigenvalue encountered!" << endl;
            return -1;
        }
    }

    Vector3d evdx = V * y;

    cout << "Solution of Eigen Value Decomposition: x = \n" << evdx << endl;

  return 0;
}