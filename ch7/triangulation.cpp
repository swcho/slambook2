#include <iostream>
#include <opencv2/opencv.hpp>
// #include "extra.h" // used in opencv2
using namespace std;
using namespace cv;

void find_feature_matches(
  const Mat &img_1, const Mat &img_2,
  std::vector<KeyPoint> &keypoints_1,
  std::vector<KeyPoint> &keypoints_2,
  std::vector<DMatch> &matches);

void pose_estimation_2d2d(
  const std::vector<KeyPoint> &keypoints_1,
  const std::vector<KeyPoint> &keypoints_2,
  const std::vector<DMatch> &matches,
  Mat &R, Mat &t);

void triangulation(
  const vector<KeyPoint> &keypoint_1,
  const vector<KeyPoint> &keypoint_2,
  const std::vector<DMatch> &matches,
  const Mat &R, const Mat &t,
  vector<Point3d> &points
);

/// 시각화용 함수
inline cv::Scalar get_color(float depth) {
  float up_th = 50, low_th = 10, th_range = up_th - low_th;
  if (depth > up_th) depth = up_th;
  if (depth < low_th) depth = low_th;
  return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}

// 픽셀 좌표를 카메라 정규화 좌표로 변환
Point2f pixel2cam(const Point2d &p, const Mat &K);

int main(int argc, char **argv) {
  if (argc != 3) {
    cout << "usage: triangulation img1 img2" << endl;
    return 1;
  }
  //-- 이미지 읽기
  Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

  vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;
  find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
  cout << "총 " << matches.size() << " 개의 매칭 쌍을 찾았습니다" << endl;

  //-- 두 이미지 간의 운동 추정
  Mat R, t;
  pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

  //-- 삼각화
  vector<Point3d> points;
  triangulation(keypoints_1, keypoints_2, matches, R, t, points);

  //-- 삼각화된 점과 특징점의 재투영 관계를 검증합니다
  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  Mat img1_plot = img_1.clone();
  Mat img2_plot = img_2.clone();
  for (int i = 0; i < matches.size(); i++) {
    // 첫 번째 이미지
    float depth1 = points[i].z;
    cout << "depth: " << depth1 << endl;
    Point2d pt1_cam = pixel2cam(keypoints_1[matches[i].queryIdx].pt, K);
    cv::circle(img1_plot, keypoints_1[matches[i].queryIdx].pt, 2, get_color(depth1), 2);

    // 두 번째 이미지
    Mat pt2_trans = R * (Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
    float depth2 = pt2_trans.at<double>(2, 0);
    cv::circle(img2_plot, keypoints_2[matches[i].trainIdx].pt, 2, get_color(depth2), 2);
  }
  cv::imshow("img 1", img1_plot);
  cv::imshow("img 2", img2_plot);
  cv::waitKey();

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
  // BFMatcher matcher ( NORM_HAMMING );
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

void pose_estimation_2d2d(
  const std::vector<KeyPoint> &keypoints_1,
  const std::vector<KeyPoint> &keypoints_2,
  const std::vector<DMatch> &matches,
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

  //-- 본질 행렬 계산
  Point2d principal_point(325.1, 249.7);        // 카메라 주점, TUM dataset 캘리브레이션 값
  int focal_length = 521;            // 카메라 초점 거리, TUM dataset 캘리브레이션 값
  Mat essential_matrix;
  essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);

  //-- 본질 행렬에서 회전 및 평행이동 정보를 복원합니다
  recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
}

void triangulation(
  const vector<KeyPoint> &keypoint_1,
  const vector<KeyPoint> &keypoint_2,
  const std::vector<DMatch> &matches,
  const Mat &R, const Mat &t,
  vector<Point3d> &points) {
  Mat T1 = (Mat_<float>(3, 4) <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0);
  Mat T2 = (Mat_<float>(3, 4) <<
    R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
    R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
    R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
  );

  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  vector<Point2f> pts_1, pts_2;
  for (DMatch m:matches) {
    // 픽셀 좌표를 카메라 좌표계로 변환
    pts_1.push_back(pixel2cam(keypoint_1[m.queryIdx].pt, K));
    pts_2.push_back(pixel2cam(keypoint_2[m.trainIdx].pt, K));
  }

  Mat pts_4d;
  cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

  // 비동차 좌표로 변환
  for (int i = 0; i < pts_4d.cols; i++) {
    Mat x = pts_4d.col(i);
    x /= x.at<float>(3, 0); // 정규화
    Point3d p(
      x.at<float>(0, 0),
      x.at<float>(1, 0),
      x.at<float>(2, 0)
    );
    points.push_back(p);
  }
}

Point2f pixel2cam(const Point2d &p, const Mat &K) {
  return Point2f
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}
