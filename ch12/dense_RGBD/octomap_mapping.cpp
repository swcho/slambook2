#include <iostream>
#include <fstream>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <octomap/octomap.h>    // for octomap

#include <Eigen/Geometry>
#include <boost/format.hpp>  // for formating strings

int main(int argc, char **argv) {
    vector<cv::Mat> colorImgs, depthImgs;    // 컬러 이미지와 깊이 이미지
    vector<Eigen::Isometry3d> poses;         // 카메라 포즈

    ifstream fin("./data/pose.txt");
    if (!fin) {
        cerr << "cannot find pose file" << endl;
        return 1;
    }

    for (int i = 0; i < 5; i++) {
        boost::format fmt("./data/%s/%d.%s"); // 이미지 파일 형식
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "png").str(), -1)); // -1 로 원본 이미지를 읽습니다

        double data[7] = {0};
        for (int i = 0; i < 7; i++) {
            fin >> data[i];
        }
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(T);
    }

    // 포인트 클라우드를 계산하고 합칩니다
    // 카메라 내부 파라미터
    double cx = 319.5;
    double cy = 239.5;
    double fx = 481.2;
    double fy = -480.0;
    double depthScale = 5000.0;

    cout << "이미지를 Octomap 으로 변환 중 ..." << endl;

    // octomap tree
    octomap::OcTree tree(0.01); // 파라미터는 해상도

    for (int i = 0; i < 5; i++) {
        cout << "이미지 변환 중: " << i + 1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];

        octomap::Pointcloud cloud;  // the point cloud in octomap

        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++) {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 깊이 값
                if (d == 0) continue; // 0 이면 측정되지 않은 것입니다
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * point;
                // 월드 좌표계의 점을 포인트 클라우드에 추가합니다
                cloud.push_back(pointWorld[0], pointWorld[1], pointWorld[2]);
            }

        // 포인트 클라우드를 팔진트리 지도에 저장합니다. 원점을 주어 투영선을 계산할 수 있습니다
        tree.insertPointCloud(cloud, octomap::point3d(T(0, 3), T(1, 3), T(2, 3)));
    }

    // 중간 노드의 점유 정보를 업데이트하고 디스크에 기록합니다
    tree.updateInnerOccupancy();
    cout << "saving octomap ... " << endl;
    tree.writeBinary("octomap.bt");
    return 0;
}
