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
