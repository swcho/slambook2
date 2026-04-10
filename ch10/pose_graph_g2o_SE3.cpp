#include <iostream>
#include <fstream>
#include <string>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

using namespace std;

/************************************************
 * 이 프로그램은 g2o solver 를 이용한 포즈 그래프 최적화 방법을 보여줍니다
 * sphere.g2o 는 인공으로 생성된 Pose graph 로, 이를 최적화합니다.
 * load 함수로 그래프 전체를 직접 읽을 수도 있지만, 코드를 직접 구현해 더 깊이 이해합니다
 * 여기서는 g2o/types/slam3d/ 의 SE3 로 포즈를 표현합니다. 실질적으로는 사원수이며 리 대수가 아닙니다.
 * **********************************************/

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Usage: pose_graph_g2o_SE3 sphere.g2o" << endl;
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
    while (!fin.eof()) {
        string name;
        fin >> name;
        if (name == "VERTEX_SE3:QUAT") {
            // SE3 정점
            g2o::VertexSE3 *v = new g2o::VertexSE3();
            int index = 0;
            fin >> index;
            v->setId(index);
            v->read(fin);
            optimizer.addVertex(v);
            vertexCnt++;
            if (index == 0)
                v->setFixed(true);
        } else if (name == "EDGE_SE3:QUAT") {
            // SE3-SE3 엣지
            g2o::EdgeSE3 *e = new g2o::EdgeSE3();
            int idx1, idx2;     // 연결된 두 정점
            fin >> idx1 >> idx2;
            e->setId(edgeCnt++);
            e->setVertex(0, optimizer.vertices()[idx1]);
            e->setVertex(1, optimizer.vertices()[idx2]);
            e->read(fin);
            optimizer.addEdge(e);
        }
        if (!fin.good()) break;
    }

    cout << "read total " << vertexCnt << " vertices, " << edgeCnt << " edges." << endl;

    cout << "optimizing ..." << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);

    cout << "saving optimization results ..." << endl;
    optimizer.save("result.g2o");

    return 0;
}
