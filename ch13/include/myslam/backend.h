//
// Created by gaoxiang on 19-5-2.
//

#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {
class Map;

/**
 * 백엔드
 * 별도의 최적화 스레드를 가지며, Map 이 업데이트될 때 최적화를 시작합니다
 * Map 업데이트는 프론트엔드가 트리거합니다
 */
class Backend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    /// 생성자에서 최적화 스레드를 시작하고 대기 상태로 둡니다
    Backend();

    // 좌우 카메라를 설정합니다. 내외부 파라미터를 얻는 데 사용합니다
    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        cam_left_ = left;
        cam_right_ = right;
    }

    /// 지도를 설정합니다
    void SetMap(std::shared_ptr<Map> map) { map_ = map; }

    /// 지도 업데이트를 트리거하고 최적화를 시작합니다
    void UpdateMap();

    /// 백엔드 스레드를 종료합니다
    void Stop();

   private:
    /// 백엔드 스레드
    void BackendLoop();

    /// 주어진 키프레임과 랜드마크 점을 최적화합니다
    void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};

}  // namespace myslam

#endif  // MYSLAM_BACKEND_H
