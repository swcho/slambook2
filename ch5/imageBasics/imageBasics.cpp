#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
  // argv[1] 로 지정된 이미지를 읽습니다
  cv::Mat image;
  image = cv::imread(argv[1]); // cv::imread 함수로 지정된 경로의 이미지를 읽습니다

  // 이미지 파일이 올바르게 읽혔는지 확인합니다
  if (image.data == nullptr) { // 데이터가 없으면 파일이 존재하지 않을 수 있습니다
    cerr << "파일 " << argv[1] << " 이 존재하지 않습니다." << endl;
    return 0;
  }

  // 파일이 정상적으로 읽혔으면 기본 정보를 출력합니다
  cout << "이미지 너비: " << image.cols << ", 높이: " << image.rows << ", 채널 수: " << image.channels() << endl;
  cv::imshow("image", image);      // cv::imshow 로 이미지를 표시합니다
  cv::waitKey(0);                  // 프로그램을 일시 정지하고 키 입력을 기다립니다

  // image 의 타입을 확인합니다
  if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
    // 이미지 타입이 요구사항에 맞지 않습니다
    cout << "컬러 이미지 또는 그레이스케일 이미지를 입력하세요." << endl;
    return 0;
  }

  // 이미지를 순회합니다. 아래 순회 방식은 임의 픽셀 접근에도 사용할 수 있습니다
  // std::chrono 를 사용하여 알고리즘 시간을 측정합니다
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  for (size_t y = 0; y < image.rows; y++) {
    // cv::Mat::ptr 로 이미지의 행 포인터를 얻습니다
    unsigned char *row_ptr = image.ptr<unsigned char>(y);  // row_ptr 은 y 번째 행의 시작 포인터
    for (size_t x = 0; x < image.cols; x++) {
      // (x, y) 위치의 픽셀에 접근합니다
      unsigned char *data_ptr = &row_ptr[x * image.channels()]; // data_ptr 은 접근할 픽셀 데이터를 가리킵니다
      // 각 채널 값을 출력합니다. 그레이스케일 이미지라면 채널이 하나입니다
      for (int c = 0; c != image.channels(); c++) {
        unsigned char data = data_ptr[c]; // data 는 I(x,y) 의 c 번째 채널 값
      }
    }
  }
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast < chrono::duration < double >> (t2 - t1);
  cout << "이미지 순회 소요 시간: " << time_used.count() << " 초." << endl;

  // cv::Mat 의 복사에 관하여
  // 직접 대입하면 데이터가 복사되지 않습니다
  cv::Mat image_another = image;
  // image_another 를 수정하면 image 도 변경됩니다
  image_another(cv::Rect(0, 0, 100, 100)).setTo(0); // 좌상단 100x100 블록을 0으로 설정
  cv::imshow("image", image);
  cv::waitKey(0);

  // clone 함수를 사용하면 데이터를 복사합니다
  cv::Mat image_clone = image.clone();
  image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
  cv::imshow("image", image);
  cv::imshow("image_clone", image_clone);
  cv::waitKey(0);

  // 이미지에 관한 다양한 기본 조작(잘라내기, 회전, 크기 조정 등)이 있으나 지면 관계상 모두 소개하기 어렵습니다. OpenCV 공식 문서를 참고하세요.
  cv::destroyAllWindows();
  return 0;
}
