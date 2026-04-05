# ch3 빌드 노트

## 환경

| 항목 | 버전 |
|---|---|
| macOS | Darwin 24.6.0 (Apple Silicon) |
| CMake | 4.1.2 |
| Compiler | Apple clang 17.0.0 |
| Eigen | 5.0.1 (Homebrew) |
| Pangolin | git `3f01854` (소스 빌드) |

---

## 빌드 방법

```bash
mkdir build && cd build
cmake -DCMAKE_PREFIX_PATH=/opt/homebrew ..
make -j4
```

---

## 수정 사항

### 1. CMake 최소 버전 (`cmake_minimum_required`)

최신 CMake(4.x)에서 `VERSION 2.8` 지원이 제거되어 오류 발생.

**적용 파일:** 루트, `useEigen/`, `useGeometry/`, `visualizeGeometry/`

```cmake
# 변경 전
cmake_minimum_required(VERSION 2.8)

# 변경 후
cmake_minimum_required(VERSION 3.5)
```

---

### 2. C++ 표준 (`CMAKE_CXX_STANDARD`)

Homebrew Eigen 5.0.1이 C++14 이상을 요구하는데 `c++11`로 설정되어 컴파일 오류 발생.

**적용 파일:** 루트 `CMakeLists.txt`, `visualizeGeometry/CMakeLists.txt`

```cmake
# 변경 전
set(CMAKE_CXX_FLAGS "-std=c++11")

# 변경 후
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

---

### 3. Eigen 헤더 경로

하드코딩된 `/usr/include/eigen3` 경로가 macOS Homebrew 환경에 존재하지 않음.  
Modern CMake target 방식으로 교체.

**적용 파일:** `useEigen/`, `useGeometry/`, `examples/`, `visualizeGeometry/`

```cmake
# 변경 전
include_directories("/usr/include/eigen3")

# 변경 후
find_package(Eigen3 REQUIRED)
target_link_libraries(<target> Eigen3::Eigen)
```

> `EIGEN3_INCLUDE_DIR` 변수는 Eigen3 cmake config에서 export되지 않음.  
> `Eigen3::Eigen` 인터페이스 타겟을 사용해야 경로가 자동으로 전달됨.

---

### 4. Pangolin 선택적 의존성

`find_package(Pangolin REQUIRED)`로 설정되어 있어 Pangolin 미설치 시 CMake 자체가 실패.  
Pangolin 없이도 나머지 타겟이 빌드되도록 조건부 처리.

**적용 파일:** `examples/CMakeLists.txt`, `visualizeGeometry/CMakeLists.txt`

```cmake
# 변경 전
find_package(Pangolin REQUIRED)
...

# 변경 후
find_package(Pangolin QUIET)
if(Pangolin_FOUND)
  ...
else()
  message(STATUS "Pangolin not found, skipping <target>")
endif()
```

---

## Pangolin 설치 (소스 빌드)

Homebrew에 C++ Pangolin 라이브러리 formula가 없어 소스에서 직접 빌드.

```bash
git clone --depth=1 https://github.com/stevenlovegrove/Pangolin.git /tmp/Pangolin
mkdir /tmp/Pangolin/build && cd /tmp/Pangolin/build
cmake -DCMAKE_PREFIX_PATH=/opt/homebrew -DCMAKE_INSTALL_PREFIX=/opt/homebrew ..
make -j4
make install
```

설치 경로: `/opt/homebrew/lib/cmake/Pangolin/PangolinConfig.cmake`

---

## 빌드 타겟

| 타겟 | 위치 | 의존성 |
|---|---|---|
| `coordinateTransform` | `examples/` | Eigen |
| `eigenMatrix` | `useEigen/` | Eigen |
| `eigenGeometry` | `useGeometry/` | Eigen |
| `plotTrajectory` | `examples/` | Eigen, Pangolin |
| `visualizeGeometry` | `visualizeGeometry/` | Eigen, Pangolin |

---

## 실행

```bash
# 좌표 변환 예제
./build/examples/coordinateTransform
# 출력: -0.0309731  0.73499  0.296108

# Pangolin GUI 예제 (터미널에서 직접 실행 필요)
./build/examples/plotTrajectory
./build/visualizeGeometry/visualizeGeometry
```

> macOS에서 Pangolin GUI는 백그라운드 실행 시 창이 표시되지 않을 수 있음.  
> 반드시 터미널에서 직접 실행할 것.
