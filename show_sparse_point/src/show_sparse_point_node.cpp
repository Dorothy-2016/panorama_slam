#include<fstream>
#include<string>
#include<vector>
#include<iostream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
#define FILE_NAME "file1"

float coef = 2.0;
using namespace std;

PointCloud::Ptr LoadTrajectory(char *filename ,int skiplines)
{
    PointCloud::Ptr cloud(new PointCloud);
    ifstream input_file;
    string s;
    input_file.open(filename,ifstream::in);
    float x,y,z;

    float traj =0;
    float lx=0,ly=0 ,lz=0;

    for (int i =0 ;i<skiplines;i++)
    {
         getline(input_file,s);
    }

    bool bfirst =true;

    float fx,fy,fz;
    while(!input_file.eof())
    {
        getline(input_file, s);
        sscanf(s.c_str(),"%f %f %f",&x,&y,&z);
        PointT p;

        if(fabs(x)>40.0||fabs(y)>40.0||fabs(z)>40.0) continue;
        p.x = x*coef;
        p.y = y*coef;
        p.z = z*coef;
        p.r = 255;
        p.b = 0;
        p.g = 0;

        traj += sqrt((x-lx)*(x-lx) +(y-ly)*(y-ly) +(z-lz)*(z-lz));

        lx = x;
        ly = y;
        lz = z;

      if(bfirst)
      {
          bfirst = false;
          fx = x;
          fy = y;
          fz = z;
         }
        cloud->points.push_back(p);

//        for (int j = 0;j<10;j++)
//        {


//        }
    }



    float error = sqrt((x-fx)*(x-fx) +(y-fy)*(y-fy) +(z-fz)*(z-fz));
    cout<<"error "<<error<<endl;
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    cout<<traj<<endl;
    return cloud;

}

int main(int argc ,char** argv)

{

    if(argc!=4)
    {
      cerr<<"args error"<<endl;
      cerr<<"usage  mappoint.txt trajectory.txt skiplines"<<endl;
      return -1;
    }

    PointCloud::Ptr Traj = LoadTrajectory(argv[2] ,atoi(argv[3]));
       ifstream input_file;
        string s ;
        input_file.open(argv[1], ifstream::in);
        PointCloud::Ptr cloud ( new PointCloud );
        while(!input_file.eof())
        {
                getline(input_file, s);
              //  cout<<s.c_str()<<endl;
                PointT p;
                float x , y , z;
                sscanf(s.c_str(),"%f %f %f",&x,&y,&z);
                if(fabs(x)>40.0||fabs(y)>40.0||fabs(z)>40.0) continue;

                p.x = x*coef;
                p.y = y*coef;
                p.z = z*coef;
                p.r = 0;
                p.g = 0;
                p.b = 255;
                cloud->points.push_back(p);

        }

            cloud->height = 1;
            cloud->width = cloud->points.size();
            cout<<"point cloud size = "<<cloud->points.size()<<endl;
            cloud->is_dense = false;


            *cloud+=*Traj;
            pcl::io::savePCDFile( "./pointcloud.pcd", *cloud );
            // 清除数据并退出
            cloud->points.clear();
            cout<<"Point cloud saved."<<endl;


     cv::Mat img = cv::imread("./buildingb_sat.png");
     cv::imshow("sat",img);

     int key =0;

     while(key!=10)

     cv::waitKey(0);
        return 0;

}
