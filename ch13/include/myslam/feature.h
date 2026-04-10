//
// Created by gaoxiang on 19-5-2.
//
#pragma once

#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include "myslam/common_include.h"

namespace myslam {

struct Frame;
struct MapPoint;

/**
 * 2D 특징점
 * 삼각화 후에는 지도 점(MapPoint)과 연결됩니다
 */
struct Feature {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;         // 이 feature 를 보유하는 프레임
    cv::KeyPoint position_;              // 2D 추출 위치
    std::weak_ptr<MapPoint> map_point_;  // 연결된 지도 점

    bool is_outlier_ = false;       // 이상치(outlier) 여부
    bool is_on_left_image_ = true;  // 왼쪽 이미지에서 추출된 경우 true, 오른쪽이면 false

   public:
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
};
}  // namespace myslam

#endif  // MYSLAM_FEATURE_H
