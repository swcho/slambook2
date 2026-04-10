# include <iostream>
# include <cmath>
# include <Eigen/Core>
# include <Eigen/Geometry>
# include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

/// 이 프로그램은 Sophus 의 기본 사용법을 보여줍니다
int main(int argc, char **argv) {
  // Z 축을 기준으로 90도 회전하는 회전 행렬
  Matrix3d R = AngleAxisd(M_PI/2,Vector3d(0,0,1)).toRotationMatrix();
  // 또는 쿼터니언
  Quaterniond q(R);
  Sophus::SO3d SO3_R(R);  // Sophus::SO3d 는 회전 행렬로 직접 구성할 수 있습니다
  Sophus::SO3d SO3_q(q);  // 쿼터니언으로도 구성할 수 있습니다
  // 두 가지는 동일합니다
  cout << "SO(3) from matrix: \n" << SO3_R.matrix() << endl;
  cout << "SO(3) from quaternion: \n" << SO3_q.matrix() << endl;
  cout << "they are equal" << endl;

  // 대수사상(로그 사상)을 사용하여 리 대수를 구합니다
  Vector3d so3 = SO3_R.log();
  cout << "so3 = " << so3.transpose() << endl;
  // hat 은 벡터를 반대칭 행렬로 변환합니다
  cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;
  // 반대로, vee 는 반대칭 행렬을 벡터로 변환합니다
  cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

  // 증분 섭동 모델로 업데이트
  Vector3d update_so3(1e-4,0,0); //假设更新量为这么多


  return 0;
}