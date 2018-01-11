#include<fstream>
#include<string>
#include<vector>
#include<iostream>
#include <stdio.h>
// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
#define FILE_NAME "file1"

using namespace std;

int main(int argc ,char** argv)

{

    if(argc!=2)
    {
      cerr<<"args error"<<endl;
      return -1;
    }
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
                p.x = x;
                p.y = y;
                p.z = z;
                cloud->points.push_back(p);

        }

        cloud->height = 1;
            cloud->width = cloud->points.size();
            cout<<"point cloud size = "<<cloud->points.size()<<endl;
            cloud->is_dense = false;
            pcl::io::savePCDFile( "./pointcloud.pcd", *cloud );
            // 清除数据并退出
            cloud->points.clear();
            cout<<"Point cloud saved."<<endl;
        return 0;

}
