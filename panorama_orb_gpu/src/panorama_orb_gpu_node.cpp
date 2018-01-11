#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iterator>
#include <iostream>
#include "ORBextractor.h"

#define PI 3.1415926

using namespace cv;
using namespace std;
float fx = 300.0;
float fy = 300.0;
float cx = 300.0;
float cy = 300.0;

using namespace  ORB_SLAM2;

void DrawORBKeyPoints(cv::Mat image ,std::vector<cv::KeyPoint> mvKeys)
{
    cout<<"Keypoint size :"<<mvKeys.size()<<endl;
    float r = 3.0;
    cv::Point2f pt1,pt2;
    cvtColor(image,image,CV_GRAY2RGB);
    for (int i = 0 ;i<mvKeys.size();i++)
    {
        pt1.x=mvKeys[i].pt.x-r;
        pt1.y=mvKeys[i].pt.y-r;
        pt2.x=mvKeys[i].pt.x+r;
        pt2.y=mvKeys[i].pt.y+r;
        cv::rectangle(image,pt1,pt2,cv::Scalar(0,255,0));
        cv::circle(image,mvKeys[i].pt,2,cv::Scalar(0,255,0),-1);

    }
    imshow("orb_features ",image);
    imwrite("./orb_extrator.png",image);
    waitKey(0);

}

Mat CreateMask(int Height,int Width)
{


 Mat tempMask = Mat::zeros(Height,Width, CV_8U);
 for (int i  = 0 ;i<Height;i++)
   {
     for (int j = 0;j<Width;j++)
     {
           int pixcor = j%(Width/4) -Width/8;
           float fai = PI/4-fabs( float(pixcor)/Width*2*PI);
           float theta =  float(-i+Height/2)/Height*PI;
           float x = -cos(theta)*sin(fai);
           float y = -sin(theta);
           float z =  cos(theta)*cos(fai);
           float u = (x/z*fx+cx) ;
           float v = (y/z*fy+cy) ;


          if(u<0.0||u>600.0||v<0.0||v>600.0)
          {
              tempMask.at<uchar>(i, j)= 0;
          }
          else
          {
              tempMask.at<uchar>(i, j)= 255;
          }

    }
 }
 return tempMask;

}

int main(int argc,char ** argv)
{
  if(argc!=3)
  {
      cerr<<"arg error  :"<<endl;

  }

  int nfeatures =  atoi(argv[2]);
  int level = 8;
  cout<<"nfeatures : "<<nfeatures<<endl;
  cout<<"level : "<<level<<endl;


  Mat image = imread(argv[1]);
  if(image.empty()){
      cerr<<"image empty"<<endl;
      return -1;
  }
   cout<<"image channels : "<<image.channels()<<endl;
   if(image.channels()==3)
    {
       cvtColor(image,image,CV_RGB2GRAY);
    }


   ORBextractor* mpORBextractor = new ORBextractor(nfeatures,1.2,level,20,7);
   std::vector<cv::KeyPoint> mvKeys;
   cv::Mat mDescriptors;

//   Mat Mask = CreateMask(image.rows,image.cols);
//   imshow("mask",Mask);
//   imwrite("mask.png",Mask);
//   cv::waitKey(0);

   for (int i = 0 ;i<3;i++)
   {
       double start_time  = cv::getTickCount();
              (*mpORBextractor)(image,cv::Mat(),mvKeys,mDescriptors);
       double duration_ms = (double(cv::getTickCount())-start_time)*1000/cv::getTickFrequency();
       cout<< " image proc time cost : "<<duration_ms <<"ms "<<endl;
   }

  DrawORBKeyPoints(image,mvKeys);


  return -1;
}
