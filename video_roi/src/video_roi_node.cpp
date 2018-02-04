#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{

    if(argc!= 2) {
       cerr<<"args error "<<endl;
       return -1;
    }
VideoCapture capture(argv[1]);
if(!capture.isOpened())
cout<<"fail to open!"<<endl;
Mat frame;
namedWindow("Extracted frame");

int count = 0 ;
char filename[100];
while(true)
{

if(!capture.read(frame))
{
cout<<"读取视频失败"<<endl;
return -1;
}
imshow("Extracted frame",frame);
cvtColor(frame,frame,CV_RGB2GRAY);
resize(frame,frame,cv::Size(1920,960));
sprintf(filename,"./rgb%d.png",count++);
imwrite(filename,frame);
waitKey(5);
}
return 0;
}
