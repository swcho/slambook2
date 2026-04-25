// Phase C spike: minimal OpenCV (4.13.0, BUILD_LIST=core,imgproc,features2d,video)
// linked statically into our myslam wasm. Validates the B′ build path:
//   1. emcmake builds OpenCV as static .a libs (Q1, Q2)
//   2. our top-level CMakeLists picks them up via find_package (Q3)
//   3. cv::goodFeaturesToTrack + cv::calcOpticalFlowPyrLK execute and produce
//      reasonable results on the KITTI mini fixture (Q4)
//
// Once the spike passes, the same translation unit pattern is reused in
// myslam/bindings/bind_features.cpp for Step 3/4.

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include <cstdint>
#include <vector>

namespace {

// Copy a Uint8Array from JS into a fresh std::vector — this owns the memory
// for the lifetime of the call. Avoids relying on HEAPU8 pointer arithmetic
// in the spike (we'll move to the fast HEAPU8.set + raw-pointer path in the
// real binding). vecFromJSArray is part of emscripten/val.h.
std::vector<std::uint8_t> copyToVector(const emscripten::val& src) {
  return emscripten::vecFromJSArray<std::uint8_t>(src);
}

// Detect Good Features To Track. `gray` is a Uint8Array of length w*h
// (1-channel, 8-bit). Returns Float64Array [x0,y0,x1,y1,...].
emscripten::val detectGFTT(emscripten::val gray, int w, int h,
                           int maxFeatures, double quality, double minDist) {
  auto pixels = copyToVector(gray);
  if ((int)pixels.size() != w * h) {
    return emscripten::val::undefined();
  }
  cv::Mat img(h, w, CV_8UC1, pixels.data());
  std::vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(img, corners, maxFeatures, quality, minDist);

  const emscripten::val Float64Array = emscripten::val::global("Float64Array");
  emscripten::val out = Float64Array.new_(corners.size() * 2);
  for (std::size_t i = 0; i < corners.size(); ++i) {
    out.set(i * 2 + 0, static_cast<double>(corners[i].x));
    out.set(i * 2 + 1, static_cast<double>(corners[i].y));
  }
  return out;
}

// Track points from `prev` to `curr` using pyramid LK. `pts` is a Float64Array
// holding [x0,y0,x1,y1,...]. Returns Float64Array [x0',y0',status0, x1',y1',status1, ...].
// status==1 ⇒ tracked, status==0 ⇒ failed.
emscripten::val trackLK(emscripten::val prev, emscripten::val curr,
                        int w, int h, emscripten::val pts) {
  auto pPrev = copyToVector(prev);
  auto pCurr = copyToVector(curr);
  if ((int)pPrev.size() != w * h || (int)pCurr.size() != w * h) {
    return emscripten::val::undefined();
  }
  cv::Mat prevMat(h, w, CV_8UC1, pPrev.data());
  cv::Mat currMat(h, w, CV_8UC1, pCurr.data());

  const std::size_t n = pts["length"].as<std::size_t>() / 2;
  std::vector<cv::Point2f> p0(n);
  for (std::size_t i = 0; i < n; ++i) {
    p0[i].x = static_cast<float>(pts[i * 2 + 0].as<double>());
    p0[i].y = static_cast<float>(pts[i * 2 + 1].as<double>());
  }
  std::vector<cv::Point2f> p1;
  std::vector<std::uint8_t> status;
  std::vector<float> err;
  cv::calcOpticalFlowPyrLK(prevMat, currMat, p0, p1, status, err,
                           cv::Size(21, 21), 3,
                           cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                            30, 0.01));

  const emscripten::val Float64Array = emscripten::val::global("Float64Array");
  emscripten::val out = Float64Array.new_(n * 3);
  for (std::size_t i = 0; i < n; ++i) {
    out.set(i * 3 + 0, static_cast<double>(p1[i].x));
    out.set(i * 3 + 1, static_cast<double>(p1[i].y));
    out.set(i * 3 + 2, static_cast<double>(status[i]));
  }
  return out;
}

// Sanity probe: returns OpenCV's reported version string. Confirms link
// against the right static libs.
std::string opencvVersion() { return CV_VERSION; }

}  // namespace

EMSCRIPTEN_BINDINGS(myslam_cv_spike) {
  emscripten::function("detectGFTT", &detectGFTT);
  emscripten::function("trackLK", &trackLK);
  emscripten::function("opencvVersion", &opencvVersion);
}
