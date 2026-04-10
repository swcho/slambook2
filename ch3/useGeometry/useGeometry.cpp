#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

// 이 프로그램은 Eigen 기하 모듈의 사용 방법을 보여줍니다

int main(int argc, char **argv) {

  // Eigen/Geometry 모듈은 다양한 회전 및 평행이동 표현을 제공합니다
  // 3D 회전 행렬은 Matrix3d 또는 Matrix3f 를 직접 사용합니다
  Matrix3d rotation_matrix = Matrix3d::Identity();
  // 회전 벡터는 AngleAxis 를 사용합니다. 내부적으로 Matrix 는 아니지만 연산자 오버로딩으로 행렬처럼 사용할 수 있습니다
  AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));     // Z 축을 기준으로 45도 회전
  cout.precision(3);
  // cout << "rotation vector = \n" << rotation_vector.transpose() << endl;
  cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;   // matrix() 로 행렬로 변환
  // 직접 대입도 가능합니다
  rotation_matrix = rotation_vector.toRotationMatrix();
  // AngleAxis 로 좌표 변환을 수행할 수 있습니다
  Vector3d v(1, 0, 0);
  Vector3d v_rotated = rotation_vector * v;
  cout << "(1,0,0) after rotation (by angle axis) = " << v_rotated.transpose() << endl;
  // 또는 회전 행렬을 사용합니다
  v_rotated = rotation_matrix * v;
  cout << "(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() << endl;

  // 오일러 각: 회전 행렬을 오일러 각으로 직접 변환할 수 있습니다
  Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX 순서, 즉 yaw-pitch-roll 순서
  cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

  // 유클리드 변환 행렬은 Eigen::Isometry 를 사용합니다
  Isometry3d T = Isometry3d::Identity();                // 3d 라고 부르지만 실제로는 4×4 행렬입니다
  T.rotate(rotation_vector);                                     // rotation_vector 에 따라 회전
  T.pretranslate(Vector3d(1, 3, 4));                     // 평행이동 벡터를 (1,3,4) 로 설정
  cout << "Transform matrix = \n" << T.matrix() << endl;

  // 변환 행렬로 좌표 변환
  Vector3d v_transformed = T * v;                              // R*v+t 에 해당
  cout << "v tranformed = " << v_transformed.transpose() << endl;

  // 아핀 변환과 사영 변환은 Eigen::Affine3d 와 Eigen::Projective3d 를 사용하면 됩니다 (생략)

  // 사원수(쿼터니언)
  // AngleAxis 를 사원수에 직접 대입할 수 있으며, 반대도 가능합니다
  Quaterniond q = Quaterniond(rotation_vector);
  cout << "quaternion from rotation vector = " << q.coeffs().transpose()
       << endl;   // coeffs 의 순서는 (x,y,z,w) 이며, w 가 실수부, 앞 세 개가 허수부입니다
  // 회전 행렬을 사원수에 대입할 수도 있습니다
  q = Quaterniond(rotation_matrix);
  cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;
  // 사원수로 벡터를 회전할 때는 오버로딩된 곱셈 연산자를 사용합니다
  v_rotated = q * v; // 수학적으로는 qvq^{-1} 입니다
  cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
  // 일반 벡터 곱으로 표현하면 아래와 같이 계산해야 합니다
  cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;

  return 0;
}
