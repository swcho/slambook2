// Phase C / Step 3 + Step 4 binding — feature detection + LK stereo matching.
//
// Extends the cv_spike API with:
//   * algorithm picker for detect (GFTT / Harris / FAST / ORB)
//   * optional mask (1-channel uint8 of size w*h, 0 = exclude — mirrors ch13's
//     20×20 exclusion around already-tracked features in DetectFeatures)
//   * tunable LK parameters (window, level, iterations, eps)
//   * optional initial-flow guesses for LK (matches ch13's
//     OPTFLOW_USE_INITIAL_FLOW + projection prior pattern in
//     FindFeaturesInRight / TrackLastFrame)
//
// Mirrors ch13/src/frontend.cpp:
//   - DetectFeatures        : algo=GFTT, num_features=150, quality=0.01, minDist=20
//   - FindFeaturesInRight   : LK pyr 11×11, 3 levels, 30 iters, eps 0.01, OPTFLOW_USE_INITIAL_FLOW
//
// Output shape:
//   detectFeatures → Float64Array [x0,y0,score0, x1,y1,score1, ...]
//   trackLK         → Float64Array [x0',y0',status0, x1',y1',status1, ...]
//
// Status semantics: 1 = tracked, 0 = lost. Matches the cv_spike contract used
// by the Step 4 UI / VerifyGate threshold.

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace myslam {

namespace {

enum DetectorKind : int {
  kGFTT = 0,
  kHarris = 1,
  kFAST = 2,
  kORB = 3,
};

std::vector<std::uint8_t> copyToVector(const emscripten::val& src) {
  return emscripten::vecFromJSArray<std::uint8_t>(src);
}

// Build a cv::Mat mask from an optional emscripten::val. If `maskVal` is
// undefined/null or wrong size, returns an empty Mat (= no mask).
cv::Mat buildMask(const emscripten::val& maskVal, int w, int h) {
  if (maskVal.isUndefined() || maskVal.isNull()) return cv::Mat();
  const std::size_t expected = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  if (maskVal["length"].as<std::size_t>() != expected) return cv::Mat();
  auto bytes = copyToVector(maskVal);
  // Copy into a heap-owned Mat — caller's vector goes out of scope after this fn.
  cv::Mat mask(h, w, CV_8UC1);
  std::memcpy(mask.data, bytes.data(), expected);
  return mask;
}

emscripten::val emptyFloat64Array() {
  return emscripten::val::global("Float64Array").new_(0);
}

// Pack [x, y, score] triples into a Float64Array. Score is detector-specific
// (Harris response for GFTT/Harris, FAST score for FAST, ORB response for ORB).
emscripten::val packKeypoints(const std::vector<cv::KeyPoint>& kps) {
  emscripten::val Float64Array = emscripten::val::global("Float64Array");
  emscripten::val out = Float64Array.new_(kps.size() * 3);
  for (std::size_t i = 0; i < kps.size(); ++i) {
    out.set(i * 3 + 0, static_cast<double>(kps[i].pt.x));
    out.set(i * 3 + 1, static_cast<double>(kps[i].pt.y));
    out.set(i * 3 + 2, static_cast<double>(kps[i].response));
  }
  return out;
}

// Same shape but built from cv::Point2f (no score available — fill with 1.0).
emscripten::val packPoints(const std::vector<cv::Point2f>& pts) {
  emscripten::val Float64Array = emscripten::val::global("Float64Array");
  emscripten::val out = Float64Array.new_(pts.size() * 3);
  for (std::size_t i = 0; i < pts.size(); ++i) {
    out.set(i * 3 + 0, static_cast<double>(pts[i].x));
    out.set(i * 3 + 1, static_cast<double>(pts[i].y));
    out.set(i * 3 + 2, 1.0);
  }
  return out;
}

}  // namespace

// detectFeatures(grayU8, w, h, algo, opts) -> Float64Array
//
// `opts` is a plain JS object. Recognised fields (all optional):
//   maxFeatures  (int)    — max points to keep (default 150)
//   qualityLevel (number) — GFTT quality threshold (default 0.01)
//   minDistance  (number) — min spacing between corners (default 20)
//   blockSize    (int)    — neighborhood size (default 3)
//   harrisK      (number) — Harris detector free param (default 0.04)
//   fastThreshold (int)   — FAST intensity threshold (default 20)
//   nonmaxSuppression (bool) — FAST non-maximal suppression (default true)
//   orbScaleFactor (number) — ORB pyramid scale (default 1.2)
//   orbNLevels   (int)    — ORB pyramid levels (default 8)
//   mask         (Uint8Array|null) — 1-channel mask of size w*h (0 = exclude)
emscripten::val detectFeatures(emscripten::val gray, int w, int h, int algo,
                               emscripten::val opts) {
  auto pixels = copyToVector(gray);
  if (static_cast<int>(pixels.size()) != w * h) return emptyFloat64Array();
  cv::Mat img(h, w, CV_8UC1, pixels.data());

  auto getInt = [&](const char* key, int def) -> int {
    if (opts.isUndefined() || opts.isNull()) return def;
    emscripten::val v = opts[key];
    return v.isUndefined() ? def : v.as<int>();
  };
  auto getNum = [&](const char* key, double def) -> double {
    if (opts.isUndefined() || opts.isNull()) return def;
    emscripten::val v = opts[key];
    return v.isUndefined() ? def : v.as<double>();
  };
  auto getBool = [&](const char* key, bool def) -> bool {
    if (opts.isUndefined() || opts.isNull()) return def;
    emscripten::val v = opts[key];
    return v.isUndefined() ? def : v.as<bool>();
  };

  const int maxFeatures = getInt("maxFeatures", 150);
  const double qualityLevel = getNum("qualityLevel", 0.01);
  const double minDistance = getNum("minDistance", 20.0);
  const int blockSize = getInt("blockSize", 3);
  const double harrisK = getNum("harrisK", 0.04);
  const int fastThreshold = getInt("fastThreshold", 20);
  const bool nonmaxSup = getBool("nonmaxSuppression", true);
  const double orbScale = getNum("orbScaleFactor", 1.2);
  const int orbLevels = getInt("orbNLevels", 8);

  cv::Mat mask = buildMask(opts.isUndefined() || opts.isNull()
                               ? emscripten::val::undefined()
                               : opts["mask"],
                           w, h);

  switch (static_cast<DetectorKind>(algo)) {
    case kGFTT:
    case kHarris: {
      const bool useHarris = (algo == kHarris);
      std::vector<cv::Point2f> corners;
      cv::goodFeaturesToTrack(img, corners, maxFeatures, qualityLevel,
                              minDistance, mask, blockSize, useHarris,
                              harrisK);
      // goodFeaturesToTrack drops the response; we recompute none and just
      // mark score as 1.0 (UI uses position only — score is informational).
      return packPoints(corners);
    }
    case kFAST: {
      std::vector<cv::KeyPoint> kps;
      cv::FAST(img, kps, fastThreshold, nonmaxSup);
      if (!mask.empty()) {
        kps.erase(std::remove_if(kps.begin(), kps.end(),
                                 [&](const cv::KeyPoint& kp) {
                                   const int x = static_cast<int>(kp.pt.x);
                                   const int y = static_cast<int>(kp.pt.y);
                                   if (x < 0 || x >= w || y < 0 || y >= h) return true;
                                   return mask.at<std::uint8_t>(y, x) == 0;
                                 }),
                  kps.end());
      }
      // Cap to maxFeatures by descending response.
      if (static_cast<int>(kps.size()) > maxFeatures) {
        std::nth_element(kps.begin(), kps.begin() + maxFeatures, kps.end(),
                         [](const cv::KeyPoint& a, const cv::KeyPoint& b) {
                           return a.response > b.response;
                         });
        kps.resize(maxFeatures);
      }
      return packKeypoints(kps);
    }
    case kORB: {
      auto orb = cv::ORB::create(maxFeatures, static_cast<float>(orbScale), orbLevels);
      std::vector<cv::KeyPoint> kps;
      orb->detect(img, kps, mask);
      return packKeypoints(kps);
    }
  }
  return emptyFloat64Array();
}

// trackLK(prevU8, currU8, w, h, prevPts, opts) -> Float64Array of triples.
//
// `prevPts` and `opts.initialPts` use the same 3-stride shape produced by
// detectFeatures: [x0,y0,score0, x1,y1,score1, ...]. The score column is
// ignored on input.
//
// `opts` recognised fields:
//   winSize     (int)    — LK window (default 11; ch13's FindFeaturesInRight uses 11)
//   maxLevel    (int)    — pyramid levels (default 3)
//   maxIter     (int)    — terminate iterations (default 30)
//   eps         (number) — terminate epsilon (default 0.01)
//   useInitialFlow (bool) — pass cv::OPTFLOW_USE_INITIAL_FLOW (default false)
//   initialPts  (Float64Array|null) — same shape as prevPts; required if useInitialFlow.
emscripten::val trackLK(emscripten::val prev, emscripten::val curr,
                        int w, int h, emscripten::val prevPts,
                        emscripten::val opts) {
  auto pPrev = copyToVector(prev);
  auto pCurr = copyToVector(curr);
  if (static_cast<int>(pPrev.size()) != w * h ||
      static_cast<int>(pCurr.size()) != w * h) {
    return emptyFloat64Array();
  }
  cv::Mat prevMat(h, w, CV_8UC1, pPrev.data());
  cv::Mat currMat(h, w, CV_8UC1, pCurr.data());

  const std::size_t total = prevPts["length"].as<std::size_t>();
  if (total % 3 != 0) return emptyFloat64Array();
  const std::size_t n = total / 3;
  std::vector<cv::Point2f> p0(n);
  for (std::size_t i = 0; i < n; ++i) {
    p0[i].x = static_cast<float>(prevPts[i * 3 + 0].as<double>());
    p0[i].y = static_cast<float>(prevPts[i * 3 + 1].as<double>());
  }

  auto getInt = [&](const char* key, int def) -> int {
    if (opts.isUndefined() || opts.isNull()) return def;
    emscripten::val v = opts[key];
    return v.isUndefined() ? def : v.as<int>();
  };
  auto getNum = [&](const char* key, double def) -> double {
    if (opts.isUndefined() || opts.isNull()) return def;
    emscripten::val v = opts[key];
    return v.isUndefined() ? def : v.as<double>();
  };
  auto getBool = [&](const char* key, bool def) -> bool {
    if (opts.isUndefined() || opts.isNull()) return def;
    emscripten::val v = opts[key];
    return v.isUndefined() ? def : v.as<bool>();
  };

  const int winSize = getInt("winSize", 11);
  const int maxLevel = getInt("maxLevel", 3);
  const int maxIter = getInt("maxIter", 30);
  const double eps = getNum("eps", 0.01);
  const bool useInitialFlow = getBool("useInitialFlow", false);

  std::vector<cv::Point2f> p1(n);
  if (useInitialFlow) {
    emscripten::val initVal = opts.isUndefined() || opts.isNull()
                                  ? emscripten::val::undefined()
                                  : opts["initialPts"];
    if (!initVal.isUndefined() && !initVal.isNull()) {
      const std::size_t initLen = initVal["length"].as<std::size_t>();
      const std::size_t mInit = (initLen % 3 == 0) ? initLen / 3 : 0;
      const std::size_t cap = std::min(n, mInit);
      for (std::size_t i = 0; i < cap; ++i) {
        p1[i].x = static_cast<float>(initVal[i * 3 + 0].as<double>());
        p1[i].y = static_cast<float>(initVal[i * 3 + 1].as<double>());
      }
      // For points beyond the supplied initial guesses, fall back to p0.
      for (std::size_t i = cap; i < n; ++i) p1[i] = p0[i];
    } else {
      // useInitialFlow set but no initialPts supplied — fall back to p0
      // (mirrors ch13's "use same pixel in left image" branch).
      for (std::size_t i = 0; i < n; ++i) p1[i] = p0[i];
    }
  }

  std::vector<std::uint8_t> status;
  std::vector<float> err;
  const int flags = useInitialFlow ? cv::OPTFLOW_USE_INITIAL_FLOW : 0;
  cv::calcOpticalFlowPyrLK(prevMat, currMat, p0, p1, status, err,
                           cv::Size(winSize, winSize), maxLevel,
                           cv::TermCriteria(cv::TermCriteria::COUNT +
                                                cv::TermCriteria::EPS,
                                            maxIter, eps),
                           flags);

  emscripten::val Float64Array = emscripten::val::global("Float64Array");
  emscripten::val out = Float64Array.new_(n * 3);
  for (std::size_t i = 0; i < n; ++i) {
    out.set(i * 3 + 0, static_cast<double>(p1[i].x));
    out.set(i * 3 + 1, static_cast<double>(p1[i].y));
    out.set(i * 3 + 2, static_cast<double>(status[i]));
  }
  return out;
}

std::string opencvVersion() { return CV_VERSION; }

}  // namespace myslam

EMSCRIPTEN_BINDINGS(myslam_features) {
  emscripten::function("detectFeatures", &myslam::detectFeatures);
  emscripten::function("trackLK", &myslam::trackLK);
  emscripten::function("opencvVersion", &myslam::opencvVersion);

  emscripten::constant("DETECTOR_GFTT", static_cast<int>(myslam::kGFTT));
  emscripten::constant("DETECTOR_HARRIS", static_cast<int>(myslam::kHarris));
  emscripten::constant("DETECTOR_FAST", static_cast<int>(myslam::kFAST));
  emscripten::constant("DETECTOR_ORB", static_cast<int>(myslam::kORB));
}
