// Minimal pinhole camera binding — Step 2 (Phase B).
//
// Mirrors ch13/include/myslam/camera.h: intrinsics (fx/fy/cx/cy) + a rigid
// stereo extrinsic. The extrinsic here is a pure translation (left:
// identity, right: shift by -baseline along x); the book uses a full Sophus
// SE3 but rotation is identity for rectified KITTI pairs and SE(3) pose
// handling is introduced in Phase E (Step 7/8). Kept header-only with no
// Eigen/Sophus dependency so the WASM binary stays tiny (~15 KB) for Step 2.
//
// Exposed API (Embind):
//   class Camera {
//     Camera(fx, fy, cx, cy, baseline, tx, ty, tz);
//     cameraToPixel(xc, yc, zc) -> [u, v]
//     pixelToCamera(u, v, depth) -> [x, y, z]
//     worldToPixel(xw, yw, zw)  -> [u, v]       (assumes T_c_w = identity)
//     pixelToWorld(u, v, depth) -> [x, y, z]    (assumes T_c_w = identity)
//   }
//   function projectBatch(camera, xyzFloat64Array) -> uvFloat64Array
//   function roundTripMaxError(camera, xyzFloat64Array, depthFloat64Array) -> number

#include <emscripten/bind.h>
#include <emscripten/val.h>
#include <array>
#include <cmath>
#include <cstddef>

namespace myslam {

class Camera {
 public:
  Camera(double fx, double fy, double cx, double cy, double baseline,
         double tx, double ty, double tz)
      : fx_(fx), fy_(fy), cx_(cx), cy_(cy),
        baseline_(baseline), tx_(tx), ty_(ty), tz_(tz) {}

  double fx() const { return fx_; }
  double fy() const { return fy_; }
  double cx() const { return cx_; }
  double cy() const { return cy_; }
  double baseline() const { return baseline_; }
  double tx() const { return tx_; }
  double ty() const { return ty_; }
  double tz() const { return tz_; }

  emscripten::val cameraToPixel(double xc, double yc, double zc) const {
    return makeVec2(fx_ * xc / zc + cx_, fy_ * yc / zc + cy_);
  }

  emscripten::val pixelToCamera(double u, double v, double depth) const {
    return makeVec3((u - cx_) * depth / fx_,
                    (v - cy_) * depth / fy_,
                    depth);
  }

  emscripten::val worldToPixel(double xw, double yw, double zw) const {
    // T_c_w = identity; rig-to-camera extrinsic is a pure translation.
    const double xc = xw + tx_;
    const double yc = yw + ty_;
    const double zc = zw + tz_;
    return cameraToPixel(xc, yc, zc);
  }

  emscripten::val pixelToWorld(double u, double v, double depth) const {
    const double xc = (u - cx_) * depth / fx_;
    const double yc = (v - cy_) * depth / fy_;
    const double zc = depth;
    return makeVec3(xc - tx_, yc - ty_, zc - tz_);
  }

 private:
  double fx_, fy_, cx_, cy_, baseline_;
  double tx_, ty_, tz_;

  static emscripten::val makeVec2(double a, double b) {
    emscripten::val out = emscripten::val::array();
    out.set(0u, a);
    out.set(1u, b);
    return out;
  }

  static emscripten::val makeVec3(double a, double b, double c) {
    emscripten::val out = emscripten::val::array();
    out.set(0u, a);
    out.set(1u, b);
    out.set(2u, c);
    return out;
  }
};

// Project a flat [x0,y0,z0, x1,y1,z1, ...] Float64Array to [u0,v0, u1,v1, ...].
// Typed-array fast path avoiding per-point emscripten::val overhead.
static emscripten::val projectBatch(const Camera& cam, emscripten::val xyz) {
  const size_t len = xyz["length"].as<size_t>();
  if (len % 3 != 0) {
    return emscripten::val::undefined();
  }
  const size_t n = len / 3;
  emscripten::val Float64Array = emscripten::val::global("Float64Array");
  emscripten::val out = Float64Array.new_(n * 2);
  for (size_t i = 0; i < n; ++i) {
    const double xc = xyz[i * 3 + 0].as<double>();
    const double yc = xyz[i * 3 + 1].as<double>();
    const double zc = xyz[i * 3 + 2].as<double>();
    out.set(i * 2 + 0, cam.fx() * xc / zc + cam.cx());
    out.set(i * 2 + 1, cam.fy() * yc / zc + cam.cy());
  }
  return out;
}

// Compute the maximum project → unproject round-trip error over a batch.
// For each (xc, yc, zc): u,v = K * (xc/zc, yc/zc); (xc',yc',zc') = unproj(u,v,zc).
// Returns max ||(xc,yc,zc) - (xc',yc',zc')||.
static double roundTripMaxError(const Camera& cam, emscripten::val xyz) {
  const size_t len = xyz["length"].as<size_t>();
  if (len % 3 != 0 || len == 0) return 0.0;
  const size_t n = len / 3;
  double maxErr = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double xc = xyz[i * 3 + 0].as<double>();
    const double yc = xyz[i * 3 + 1].as<double>();
    const double zc = xyz[i * 3 + 2].as<double>();
    if (zc <= 0.0) continue;
    const double u = cam.fx() * xc / zc + cam.cx();
    const double v = cam.fy() * yc / zc + cam.cy();
    const double xcR = (u - cam.cx()) * zc / cam.fx();
    const double ycR = (v - cam.cy()) * zc / cam.fy();
    const double dx = xc - xcR;
    const double dy = yc - ycR;
    const double err = std::hypot(dx, dy);
    if (err > maxErr) maxErr = err;
  }
  return maxErr;
}

}  // namespace myslam

EMSCRIPTEN_BINDINGS(myslam_camera) {
  using emscripten::class_;
  using emscripten::function;

  class_<myslam::Camera>("Camera")
      .constructor<double, double, double, double, double, double, double, double>()
      .property("fx", &myslam::Camera::fx)
      .property("fy", &myslam::Camera::fy)
      .property("cx", &myslam::Camera::cx)
      .property("cy", &myslam::Camera::cy)
      .property("baseline", &myslam::Camera::baseline)
      .property("tx", &myslam::Camera::tx)
      .property("ty", &myslam::Camera::ty)
      .property("tz", &myslam::Camera::tz)
      .function("cameraToPixel", &myslam::Camera::cameraToPixel)
      .function("pixelToCamera", &myslam::Camera::pixelToCamera)
      .function("worldToPixel", &myslam::Camera::worldToPixel)
      .function("pixelToWorld", &myslam::Camera::pixelToWorld);

  function("projectBatch", &myslam::projectBatch);
  function("roundTripMaxError", &myslam::roundTripMaxError);
}
