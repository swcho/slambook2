#include <emscripten/bind.h>
#include <string>

namespace myslam {

std::string greet(const std::string& who) {
  return "hello from wasm, " + who + "!";
}

int add(int a, int b) {
  return a + b;
}

}  // namespace myslam

EMSCRIPTEN_BINDINGS(hello) {
  emscripten::function("greet", &myslam::greet);
  emscripten::function("add", &myslam::add);
}
