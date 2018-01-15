#include <iostream>

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>
using namespace std;

void BA_test()
{
      // 步骤1：初始化g2o优化器
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);


    long unsigned int maxKFid = 0;



    for(size_t i=0; i<1; i++)
    {

        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();

        Eigen:: Quaterniond q;
        g2o::SE3Quat a;

        vSE3->setEstimate(a);
        cout<<vSE3->estimate()<<"fucking "<<endl;
        vSE3->setId(0);
        vSE3->setFixed(i==0);
        optimizer.addVertex(vSE3);
        if(i>maxKFid)
            maxKFid=i;
    }




    // Set MapPoint vertices
    // 步骤2.2：向优化器添加MapPoints顶点
    int i = 0;

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Eigen::Vector3d(3,3,3));
        const int id = i+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);


                Eigen::Matrix<double,2,1> obs;
                obs << 600, 600;
                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(vPoint));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));

                e->setMeasurement(obs);
//                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity());
                e->fx = 300;
                e->fy = 300;
                e->cx = 300;
                e->cy = 300;

               optimizer.addEdge(e);


    // Optimize!
    // 步骤4：开始优化
    optimizer.initializeOptimization();
    optimizer.optimize(3);
   
}
int main()
{
cout<<"test"<<endl;
BA_test();
return -1 ;
}
