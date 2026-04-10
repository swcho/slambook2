#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// #include "extra.h" // use this if in OpenCV2

using namespace std;
using namespace cv;

/****************************************************
 * 이 프로그램은 2D-2D 특징점 매칭으로 카메라 운동을 추정하는 방법을 보여줍니다
 * **************************************************/

void find_feature_matches(
  const Mat &img_1, const Mat &img_2,
  std::vector<KeyPoint> &keypoints_1,
  std::vector<KeyPoint> &keypoints_2,
  std::vector<DMatch> &matches);

void pose_estimation_2d2d(
  std::vector<KeyPoint> keypoints_1,
  std::vector<KeyPoint> keypoints_2,
  std::vector<DMatch> matches,
  Mat &R, Mat &t);

// 픽셀 좌표를 카메라 정규화 좌표로 변환
Point2d pixel2cam(const Point2d &p, const Mat &K);

int main(int argc, char **argv) {
  if (argc != 3) {
    cout << "usage: pose_estimation_2d2d img1 img2" << endl;
    return 1;
  }
  //-- 이미지 읽기
  Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
  assert(img_1.data && img_2.data && "Can not load images!");

  vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;
  find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
  cout << "총 " << matches.size() << " 개의 매칭 쌍을 찾았습니다" << endl;

  //-- 두 이미지 간의 운동 추정
  Mat R, t;
  pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

  //-- E=t^R*scale 검증
  Mat t_x =
    (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
      t.at<double>(2, 0), 0, -t.at<double>(0, 0),
      -t.at<double>(1, 0), t.at<double>(0, 0), 0);

  cout << "t^R=" << endl << t_x * R << endl;

  //-- 에피폴라 제약 조건 검증
  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  for (DMatch m: matches) {
    Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
    Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
    Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
    Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
    Mat d = y2.t() * t_x * R * y1;
    cout << "epipolar constraint = " << d << endl;
  }
  return 0;
}

void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &keypoints_1,
                          std::vector<KeyPoint> &keypoints_2,
                          std::vector<DMatch> &matches) {
  //-- 초기화
  Mat descriptors_1, descriptors_2;
  // used in OpenCV3
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  // use this if you are in OpenCV2
  // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
  // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  //-- 1단계: Oriented FAST 코너점 위치를 검출합니다
  detector->detect(img_1, keypoints_1);
  detector->detect(img_2, keypoints_2);

  //-- 2단계: 코너점 위치를 기반으로 BRIEF 디스크립터를 계산합니다
  descriptor->compute(img_1, keypoints_1, descriptors_1);
  descriptor->compute(img_2, keypoints_2, descriptors_2);

  //-- 3단계: 두 이미지의 BRIEF 디스크립터를 Hamming 거리로 매칭합니다
  vector<DMatch> match;
  //BFMatcher matcher ( NORM_HAMMING );
  matcher->match(descriptors_1, descriptors_2, match);

  //-- 4단계: 매칭 쌍을 필터링합니다
  double min_dist = 10000, max_dist = 0;

  // 모든 매칭의 최소 거리와 최대 거리를 찾습니다 (가장 유사한 쌍과 가장 유사하지 않은 쌍의 거리)
  for (int i = 0; i < descriptors_1.rows; i++) {
    double dist = match[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist);
  printf("-- Min dist : %f \n", min_dist);

  // 디스크립터 간 거리가 최솟값의 두 배보다 크면 잘못된 매칭으로 간주합니다. 최솟값이 매우 작을 경우를 대비해 경험적 하한값으로 30을 사용합니다.
  for (int i = 0; i < descriptors_1.rows; i++) {
    if (match[i].distance <= max(2 * min_dist, 30.0)) {
      matches.push_back(match[i]);
    }
  }
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
  return Point2d
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

void pose_estimation_2d2d(std::vector<KeyPoint> keypoints_1,
                          std::vector<KeyPoint> keypoints_2,
                          std::vector<DMatch> matches,
                          Mat &R, Mat &t) {
  // 카메라 내부 파라미터, TUM Freiburg2
  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

  //-- 매칭 점을 vector<Point2f> 형태로 변환합니다
  vector<Point2f> points1;
  vector<Point2f> points2;

  for (int i = 0; i < (int) matches.size(); i++) {
    points1.push_back(keypoints_1[matches[i].queryIdx].pt);
    points2.push_back(keypoints_2[matches[i].trainIdx].pt);
  }

  //-- 기초 행렬 계산
  Mat fundamental_matrix;
  fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);
  cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

  //-- 본질 행렬 계산
  Point2d principal_point(325.1, 249.7);  // 카메라 주점, TUM dataset 캘리브레이션 값
  double focal_length = 521;      // 카메라 초점 거리, TUM dataset 캘리브레이션 값
  Mat essential_matrix;
  essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
  cout << "essential_matrix is " << endl << essential_matrix << endl;

  //-- 단응 행렬 계산
  //-- 이 예시의 장면은 평면이 아니므로 단응 행렬의 의미가 크지 않습니다
  Mat homography_matrix;
  homography_matrix = findHomography(points1, points2, RANSAC, 3);
  cout << "homography_matrix is " << endl << homography_matrix << endl;

  //-- 본질 행렬에서 회전 및 평행이동 정보를 복원합니다
  // 이 함수는 OpenCV3 에서만 제공됩니다
  recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
  cout << "R is " << endl << R << endl;
  cout << "t is " << endl << t << endl;

}
