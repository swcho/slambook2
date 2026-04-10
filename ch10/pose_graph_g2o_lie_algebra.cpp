#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <sophus/se3.hpp>

using namespace std;
using namespace Eigen;
using Sophus::SE3d;
using Sophus::SO3d;

/************************************************
 * 이 프로그램은 g2o solver 를 이용한 포즈 그래프 최적화 방법을 보여줍니다
 * sphere.g2o 는 인공으로 생성된 Pose graph 로, 이를 최적화합니다.
 * load 함수로 그래프 전체를 직접 읽을 수도 있지만, 코드를 직접 구현해 더 깊이 이해합니다
 * 이 절에서는 리 대수로 포즈 그래프를 표현하며, 노드와 엣지 방식은 사용자 정의합니다
 * **********************************************/

typedef Matrix<double, 6, 6> Matrix6d;

// 주어진 오차로 J_R^{-1} 의 근삿값을 구합니다
Matrix6d JRInv(const SE3d &e) {
    Matrix6d J;
    J.block(0, 0, 3, 3) = SO3d::hat(e.so3().log());
    J.block(0, 3, 3, 3) = SO3d::hat(e.translation());
    J.block(3, 0, 3, 3) = Matrix3d::Zero(3, 3);
    J.block(3, 3, 3, 3) = SO3d::hat(e.so3().log());
    // J = J * 0.5 + Matrix6d::Identity();
    J = Matrix6d::Identity();    // try Identity if you want
    return J;
}

// 리 대수 정점
typedef Matrix<double, 6, 1> Vector6d;

class VertexSE3LieAlgebra : public g2o::BaseVertex<6, SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(istream &is) override {
        double data[7];
        for (int i = 0; i < 7; i++)
            is >> data[i];
        setEstimate(SE3d(
            Quaterniond(data[6], data[3], data[4], data[5]),
            Vector3d(data[0], data[1], data[2])
        ));
    }

    virtual bool write(ostream &os) const override {
        os << id() << " ";
        Quaterniond q = _estimate.unit_quaternion();
        os << _estimate.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << endl;
        return true;
    }

    virtual void setToOriginImpl() override {
        _estimate = SE3d();
    }

    // 좌곱셈으로 업데이트
    virtual void oplusImpl(const double *update) override {
        Vector6d upd;
        upd << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = SE3d::exp(upd) * _estimate;
    }
};

// 두 리 대수 노드 사이의 엣지
class EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6, SE3d, VertexSE3LieAlgebra, VertexSE3LieAlgebra> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual bool read(istream &is) override {
        double data[7];
        for (int i = 0; i < 7; i++)
            is >> data[i];
        Quaterniond q(data[6], data[3], data[4], data[5]);
        q.normalize();
        setMeasurement(SE3d(q, Vector3d(data[0], data[1], data[2])));
        for (int i = 0; i < information().rows() && is.good(); i++)
            for (int j = i; j < information().cols() && is.good(); j++) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    virtual bool write(ostream &os) const override {
        VertexSE3LieAlgebra *v1 = static_cast<VertexSE3LieAlgebra *> (_vertices[0]);
        VertexSE3LieAlgebra *v2 = static_cast<VertexSE3LieAlgebra *> (_vertices[1]);
        os << v1->id() << " " << v2->id() << " ";
        SE3d m = _measurement;
        Eigen::Quaterniond q = m.unit_quaternion();
        os << m.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << " ";

        // information matrix
        for (int i = 0; i < information().rows(); i++)
            for (int j = i; j < information().cols(); j++) {
                os << information()(i, j) << " ";
            }
        os << endl;
        return true;
    }

    // 오차 계산 (책의 유도와 일치)
    virtual void computeError() override {
        SE3d v1 = (static_cast<VertexSE3LieAlgebra *> (_vertices[0]))->estimate();
        SE3d v2 = (static_cast<VertexSE3LieAlgebra *> (_vertices[1]))->estimate();
        _error = (_measurement.inverse() * v1.inverse() * v2).log();
    }

    // 야코비 계산
    virtual void linearizeOplus() override {
        SE3d v1 = (static_cast<VertexSE3LieAlgebra *> (_vertices[0]))->estimate();
        SE3d v2 = (static_cast<VertexSE3LieAlgebra *> (_vertices[1]))->estimate();
        Matrix6d J = JRInv(SE3d::exp(_error));
        // J 를 단위 행렬로 근사해도 됩니다
        _jacobianOplusXi = -J * v2.inverse().Adj();
        _jacobianOplusXj = J * v2.inverse().Adj();
    }
};

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Usage: pose_graph_g2o_SE3_lie sphere.g2o" << endl;
        return 1;
    }
    ifstream fin(argv[1]);
    if (!fin) {
        cout << "file " << argv[1] << " does not exist." << endl;
        return 1;
    }

    // g2o 설정
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
    typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // 그래프 모델
    optimizer.setAlgorithm(solver);   // 솔버 설정
    optimizer.setVerbose(true);       // 디버그 출력 활성화

    int vertexCnt = 0, edgeCnt = 0; // 정점과 엣지의 수

    vector<VertexSE3LieAlgebra *> vectices;
    vector<EdgeSE3LieAlgebra *> edges;
    while (!fin.eof()) {
        string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {
            // 정점
            VertexSE3LieAlgebra *v = new VertexSE3LieAlgebra();
            int index = 0;
            fin >> index;
            v->setId(index);
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            vectices.push_back(v);
            if (index == 0)
                v->setFixed(true);
        } else if (name == "EDGE_SE3:QUAT") {
            // SE3-SE3 엣지
            EdgeSE3LieAlgebra *e = new EdgeSE3LieAlgebra();
            int idx1, idx2;     // 연결된 두 정점
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
            edges.push_back(e);
        }
        if (!fin.good()) break;
    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    cout << "saving optimization results ..." << endl;

    // 사용자 정의 정점을 g2o 에 등록하지 않았으므로 직접 저장합니다
    // SE3 정점과 엣지로 위장하여 g2o_viewer 에서 인식할 수 있도록 합니다
    ofstream fout("result_lie.g2o");
    for (VertexSE3LieAlgebra *v:vectices) {
        fout << "VERTEX_SE3:QUAT ";
        v->write(fout);
    }
    for (EdgeSE3LieAlgebra *e:edges) {
        fout << "EDGE_SE3:QUAT ";
        e->write(fout);
    }
    fout.close();
    return 0;
}
