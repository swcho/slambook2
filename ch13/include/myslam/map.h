#pragma once
#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam {

/**
 * @brief 지도
 * 지도와의 상호작용: 프론트엔드는 InsertKeyframe 과 InsertMapPoint 를 호출해 새 프레임과 지도 점을 삽입하고,
 * 백엔드는 지도 구조를 유지하며 outlier 를 판별하고 제거합니다
 */
class Map {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

    Map() {}

    /// 키프레임을 추가합니다
    void InsertKeyFrame(Frame::Ptr frame);
    /// 지도 점(landmark)을 추가합니다
    void InsertMapPoint(MapPoint::Ptr map_point);

    /// 모든 지도 점을 가져옵니다
    LandmarksType GetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }
    /// 모든 키프레임을 가져옵니다
    KeyframesType GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    /// 활성화된 지도 점을 가져옵니다
    LandmarksType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    /// 활성화된 키프레임을 가져옵니다
    KeyframesType GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    /// 관측 횟수가 0인 점을 지도에서 제거합니다
    void CleanMap();

   private:
    // 오래된 키프레임을 비활성 상태로 만듭니다
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;         // all landmarks
    LandmarksType active_landmarks_;  // active landmarks
    KeyframesType keyframes_;         // all key-frames
    KeyframesType active_keyframes_;  // all key-frames

    Frame::Ptr current_frame_ = nullptr;

    // settings
    int num_active_keyframes_ = 7;  // 활성화된 키프레임 수
};
}  // namespace myslam

#endif  // MAP_H
