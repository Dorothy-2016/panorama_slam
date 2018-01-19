#include <iostream>
#include <string>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <fstream>
#include <boost/thread/thread.hpp>
//PCL

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>

//G2O

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include<Eigen/StdVector>

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;


using namespace cv;
using namespace std;

float scale = 0.5 ;
struct CorrespondPoint
{
    cv::Point2f pt1;
    cv::Point2f pt2;
    CorrespondPoint(cv::Point2f point1 ,cv::Point2f point2 ): pt1(point1),pt2(point2){}
};
std::vector<CorrespondPoint>   Cp;
std::vector<PointT> SparsePoint;
cv::Mat R ,t;
void ReadSparsePoint(const string filename)
{
    ifstream input_file;
           string s ;
           input_file.open(filename.c_str(), ifstream::in);

           while(!input_file.eof())
           {
                   getline(input_file, s);
                 //  cout<<s.c_str()<<endl;
                   PointT p;
                   float x , y , z;
                   sscanf(s.c_str(),"%f %f %f",&x,&y,&z);
                   p.x = x;
                   p.y = y;
                   p.z = z;
                   SparsePoint.push_back(p);

           }
          SparsePoint.pop_back();
   input_file.close();
   cout<<"read "<<SparsePoint.size()<<" sparse points"<<endl;

}
void ReadKeyPointData(const string filename)
{
    ifstream keyPointFile;
    keyPointFile.open(filename.c_str(),ifstream::in);
    Cp.clear();
    string s ;
    float x1 ,y1 ,x2,y2;
    while(!keyPointFile.eof())
     {
        getline(keyPointFile, s);
        sscanf(s.c_str(),"%f %f %f %f",&x1,&y1,&x2,&y2);
        cv::Point2f pt1,pt2;
        pt1.x = x1*scale;
        pt1.y = y1*scale;
        pt2.x = x2*scale;
        pt2.y = y2*scale;
        Cp.push_back(CorrespondPoint(pt1,pt2));
      }
    keyPointFile.close();
    Cp.pop_back();
    cout<<"find correspond points "<<Cp.size()<<endl;
}
void ReadRT(string filename)
{
    ifstream poseFile;
    poseFile.open(filename.c_str());
    R = cv::Mat::zeros(3,3,CV_32F);
    t = cv::Mat::zeros(3,1,CV_32F);
    float  v1,v2,v3;
    string s ;
    for(int i = 0 ;i<3;i++)
    {
        getline(poseFile, s);
        sscanf(s.c_str(),"%f %f %f ",&v1,&v2,&v3);
        R.at<float>(i,0) = v1;
        R.at<float>(i,1) = v2;
        R.at<float>(i,2) = v3;
    }
    getline(poseFile, s);
    sscanf(s.c_str(),"%f %f %f ",&v1,&v2,&v3);
    t.at<float>(0,0) = v1;
    t.at<float>(1,0) = v2;
    t.at<float>(2,0) = v3;

}
PointCloud::Ptr CreatePointCloud(int index)
{
    //set the index correspond pointcloud red

    PointCloud::Ptr cloud (new PointCloud);
    if(SparsePoint.size()==0) return cloud;

    for (int i = 0 ;i<SparsePoint.size();i++)
    {

        PointT p = SparsePoint[i];
        if(i == index )
        {
            p.r = 255;
            p.g = 0;
            p.b = 0;
        }
        else
        {
            p.r = 0;
            p.g = 255;
            p.b = 0;

        }
        cloud->points.push_back(p);

    }
    cloud->height = 1;
               cloud->width = cloud->points.size();
               cloud->is_dense = false;
     return cloud;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (PointCloud::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer->addPointCloud<PointT> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
g2o::SE3Quat toSE3Quat(const cv::Mat &R,const cv::Mat t)
{
    Eigen::Matrix<double,3,3> R1;
    R1 << R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2),
         R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2),
         R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2);

    Eigen::Matrix<double,3,1> t1(t.at<float>(0,0), t.at<float>(1,0), t.at<float>(2,0));

    return g2o::SE3Quat(R1,t1);
}

Eigen::Matrix<double,3,1> toVector3d(const PointT point)
{
    Eigen::Matrix<double,3,1> v;
    v << point.x,point.y,point.z;
    return v;
}

void Optimizer()
{

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(toSE3Quat(R,t));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    std::vector<g2o::EdgeSE3ProjectXYZOnlyPose_Panoramic*> edges;

    for (int i = 0 ;i<SparsePoint.size();i++)
    {

        Eigen::Matrix<double,2,1> obs;
        cv::Point2f pt = Cp[i].pt2;
        obs <<pt.x*2,pt.y*2;


        g2o::EdgeSE3ProjectXYZOnlyPose_Panoramic* e = new g2o::EdgeSE3ProjectXYZOnlyPose_Panoramic();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e->setMeasurement(obs);
        e->setInformation(Eigen::Matrix2d::Identity());

//        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//        e->setRobustKernel(rk);


        e->fx = 300.0;
        e->fy = 300.0;
        e->cx = 300.0;
        e->cy = 300.0;
        PointT p = SparsePoint[i];
        e->Xw[0] = p.x;
        e->Xw[1] = p.y;
        e->Xw[2] = p.z;
        optimizer.addEdge(e);
        edges.push_back(e);
    }

    float  errorSum = 0;
    for (int i = 0;i<edges.size();i++)
    {
        errorSum+= edges[i]->getEdgeError();
    }
    optimizer.initializeOptimization();
    cout<<"before optimization --"<<endl;
    cout<<"total error "<<endl<<errorSum<<endl;


    optimizer.optimize(100);


    float errorSum2 = 0;
    for (int i = 0;i<edges.size();i++)
    {
        errorSum2+= edges[i]->getEdgeError();
    }
    cout<<"after optimization --"<<endl;
    cout<<"total error "<<endl<<errorSum2<<endl;

}
void GlobalBundleAdjustment()
{
//    vector<bool> vbNotIncludedMP;
//    vbNotIncludedMP.resize(vpMP.size());

    // 步骤1：初始化g2o优化器
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    for(size_t i=0; i<2; i++)
    {

        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        if(i==0)
           {
            vSE3->setEstimate(g2o::SE3Quat());
            vSE3->setFixed(true);
        }
        else
        {
            vSE3->setEstimate(toSE3Quat(R,t));
            vSE3->setFixed(false);
        }
        vSE3->setId(i);
        optimizer.addVertex(vSE3);
    }

    const float thHuber2D = sqrt(5.99);


    for(int i=0; i<SparsePoint.size(); i++)
    {
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(toVector3d(SparsePoint[i]));
        vPoint->setId(i+2);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
     }

     vector<g2o::EdgeSE3ProjectXYZ_Panoramic*> edges;

     for (int i = 0;i<SparsePoint.size();i++)
     {
        for(int k = 0;k<2;k++)
        {
        g2o::EdgeSE3ProjectXYZ_Panoramic* e = new g2o::EdgeSE3ProjectXYZ_Panoramic();
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(i+2)));

        Eigen::Matrix<double,2,1> obs;
            if(k == 0 )
            {
               cv::Point2f pt = Cp[i].pt1;
               obs <<pt.x*2,pt.y*2;
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            }
            else
            {
                cv::Point2f pt = Cp[i].pt2;
                obs <<pt.x*2,pt.y*2;
                 e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(1)));
            }
            e->setMeasurement(obs);
            e->setInformation(Eigen::Matrix2d::Identity());
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber2D);
            optimizer.addEdge(e);
            e->fx = 300.0;
            e->fy = 300.0;
            e->cx = 300.0;
            e->cy = 300.0;
            edges.push_back(e);
        }
     }

        float  errorSum = 0;
        for (int i = 0;i<edges.size();i++)
        {
            errorSum+= edges[i]->getEdgeError();
        }
        optimizer.initializeOptimization();
        cout<<"before optimization "<<endl;
        cout<<"total error "<<endl<<errorSum<<endl;


        optimizer.optimize(100);


        float errorSum2 = 0;
        for (int i = 0;i<edges.size();i++)
        {
            errorSum2+= edges[i]->getEdgeError();
        }
        cout<<"after optimization"<<endl;
        cout<<"total error "<<endl<<errorSum2<<endl;


    //Keyframes

        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(1));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        cout<<SE3quat.to_homogeneous_matrix()<<endl;
       int nPoints  = SparsePoint.size();
       SparsePoint.clear();
       for (int i =0 ;i<nPoints;i++)
       {
          g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i+2));
          Eigen::Vector3d  p = vPoint->estimate();
          PointT pclPoint ;
          pclPoint.x = p[0];
          pclPoint.y = p[1];
          pclPoint.z = p[2];
          SparsePoint.push_back(pclPoint);

       }
}


int main(int argc ,char ** argv)
{

    if(argc!=4)
    {
        cerr<<"args error "<<endl;
        cout<< "usage keypointfile mappointfile id_of_firstframe"<<endl;
        return -1;
    }



    ReadKeyPointData(string(argv[1]));
    ReadSparsePoint(string(argv[2]));
    ReadRT(string("pose.txt"));


    GlobalBundleAdjustment();
//    Optimizer();
    cout<<R<<endl;
    cout<<t<<endl;
    char filename[100];
    int frameId =  atoi(argv[3]);
    sprintf(filename,"frames/rgb%d.png",frameId);
    Mat image1 = imread(filename);
    frameId = frameId+5;
    sprintf(filename,"frames/rgb%d.png",frameId);
    Mat image2 = imread(filename);
    cv::Size newSz;
    newSz.width = ceil(scale*image1.cols);
    newSz.height  = ceil(scale*image1.rows);
    cv::resize(image1,image1,newSz);
    cv::resize(image2,image2,newSz);
    Mat wholeImage(image1.rows*2,image1.cols, CV_8UC3);
    image1.copyTo ( wholeImage( cv::Rect ( 0,0,image1.cols,image1.rows ) ) );
    image2.copyTo ( wholeImage( cv::Rect ( 0,image1.rows,image1.cols,image1.rows ) ) );

    imshow("Correspond",wholeImage);
    cv::waitKey(0);
     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
     viewer->setBackgroundColor (0,0,0);
     viewer->addCoordinateSystem (1.0);
     viewer->initCameraParameters ();

//    pcl::visualization::CloudViewer viewer("viewer");


    for (int  i = 0;i<Cp.size();i++)
    {
        Point2f pt1 ,pt2;
        CorrespondPoint Cpp =  Cp[i];
        pt1 = Cpp.pt1;
        pt2 = Cpp.pt2;

        Point2d drawPt1;
        Point2d drawPt2;
        drawPt1.x = pt1.x;
        drawPt1.y = pt1.y;
        drawPt2.x = pt2.x;
        drawPt2.y = pt2.y+double(image1.rows);


        float b = 255*float ( rand() ) /RAND_MAX;
        float g = 255*float ( rand() ) /RAND_MAX;
        float r = 255*float ( rand() ) /RAND_MAX;

        Mat tempImg  =  wholeImage.clone();
        cv::circle (tempImg, drawPt1, 8, cv::Scalar ( b,g,r ), 5 );
        cv::circle (tempImg, drawPt2, 8, cv::Scalar ( b,g,r ), 5 );
        cv::line (tempImg, drawPt1,drawPt2, cv::Scalar ( b,g,r ), 3 );

        PointCloud::Ptr cloud =  CreatePointCloud(i);
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
        viewer->addPointCloud<PointT> (cloud, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10,"sample cloud");

        bool bloop = true;
        while (!viewer->wasStopped()&&bloop)
          {
            imshow("match",tempImg);
            viewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
            char key = cv::waitKey(10);
           if(int(key)!=-1) bloop= false;
          }
        viewer->removePointCloud("sample cloud");


        //        viewer.showCloud( cloud );


    }
    std::cout<<"hello"<<std::endl;

return -1;
}
