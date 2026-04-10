#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H
#include "myslam/camera.h"
#include "myslam/common_include.h"
#include "myslam/frame.h"

namespace myslam {

/**
 * 데이터셋 읽기
 * 생성 시 설정 파일 경로를 전달합니다. 설정 파일의 dataset_dir 이 데이터셋 경로입니다
 * Init 후에는 카메라와 다음 프레임 이미지를 가져올 수 있습니다
 */
class Dataset {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;
    Dataset(const std::string& dataset_path);

    /// 초기화, 성공 여부를 반환합니다
    bool Init();

    /// create and return the next frame containing the stereo images
    Frame::Ptr NextFrame();

    /// get camera by id
    Camera::Ptr GetCamera(int camera_id) const {
        return cameras_.at(camera_id);
    }

   private:
    std::string dataset_path_;
    int current_image_index_ = 0;

    std::vector<Camera::Ptr> cameras_;
};
}  // namespace myslam

#endif
