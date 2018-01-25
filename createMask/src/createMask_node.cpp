#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace std;
using namespace cv;


int main(int argc ,char ** argv)
{

    if(argc!=5)
    {
      cerr<<"args error "<<endl;
      cerr<<"Usage :  minX maxX  minY maxY"<<endl;
    }
    int minX = atoi(argv[1]);
    int maxX = atoi(argv[2]);
    int minY = atoi(argv[3]);
    int maxY = atoi(argv[4]);
    int Height = 960;
    int Width  = 1920 ;
    Mat tempMask = Mat::zeros(Height,Width, CV_8U);
    for (int i  = 0 ;i<Height;i++)
      {
        for (int j = 0;j<Width;j++)
        {
             if(j<minX||j>maxX||i<minY||i>maxY)     tempMask.at<uchar>(i, j)= 0;
             else     tempMask.at<uchar>(i, j)= 255;

       }
    }
    imshow("mask",tempMask);
    waitKey(0);
    imwrite("./mask.png",tempMask);
    cout<<"create mask success"<<endl;
    return 1;
}
