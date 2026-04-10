#include <opencv2/opencv.hpp>
#include <string>

using namespace std;

string image_file = "./distorted.png";   // 경로가 올바른지 확인하세요

int main(int argc, char **argv) {

  // 이 프로그램은 왜곡 보정 코드를 구현합니다. OpenCV 의 왜곡 보정 함수를 호출할 수도 있지만, 직접 구현해 보면 이해에 도움이 됩니다.
  // 왜곡 파라미터
  double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
  // 내부 파라미터
  double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

  cv::Mat image = cv::imread(image_file, 0);   // 이미지는 그레이스케일, CV_8UC1
  int rows = image.rows, cols = image.cols;
  cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1);   // 왜곡 보정된 이미지

  // 왜곡 보정 후 이미지의 내용을 계산합니다
  for (int v = 0; v < rows; v++) {
    for (int u = 0; u < cols; u++) {
      // 공식에 따라 점(u,v) 가 왜곡 이미지에서 대응하는 좌표(u_distorted, v_distorted) 를 계산합니다
      double x = (u - cx) / fx, y = (v - cy) / fy;
      double r = sqrt(x * x + y * y);
      double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
      double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
      double u_distorted = fx * x_distorted + cx;
      double v_distorted = fy * y_distorted + cy;

      // 값 대입 (최근접 이웃 보간)
      if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
        image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
      } else {
        image_undistort.at<uchar>(v, u) = 0;
      }
    }
  }

  // 왜곡 보정된 이미지를 표시합니다
  cv::imshow("distorted", image);
  cv::imshow("undistorted", image_undistort);
  cv::waitKey();
  return 0;
}
