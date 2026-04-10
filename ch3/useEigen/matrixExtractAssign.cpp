# include <iostream>
# include <Eigen/Core>

#include <random> // C++11 난수 라이브러리 사용




using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 5

int main(int argc, char **argv) {
    // // 设置随机种子， time（0）作为随机种子不安全
    // srand(static_cast<unsigned int>(time(0)));

    std::random_device rd;                          // 고품질 난수 시드 획득
    std::default_random_engine generator(rd());     // 난수 생성기 초기화
    std::uniform_real_distribution<double> distribution(-1.0, 1.0); // 균등 분포 [-1, 1]


    cout.precision(3);

    // Eigen 행렬 초기화
    MatrixXd bigMatrix(MATRIX_SIZE, MATRIX_SIZE); // rows x cols 크기의 행렬 생성

    // 3. 난수로 행렬 채우기
    for (int i = 0; i < MATRIX_SIZE; ++i) {
        for (int j = 0; j < MATRIX_SIZE; ++j) {
            bigMatrix(i, j) = distribution(generator); // 각 원소를 난수로 채우기
        }
    }

    cout << "The big matrix: \n" << bigMatrix << endl;

    Matrix3d extractedBlock = bigMatrix.block<3,3>(0,0);

    cout << "The extracted matrix block: \n" << extractedBlock << endl;

    extractedBlock.setIdentity();

    cout << "The assigned matrix block: \n" << extractedBlock << endl;

    return 0;
}
