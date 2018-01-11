#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#define PI 3.141592657
using namespace std;
using namespace cv;
int srcRows;
int srcCols;
float fx = 300.0;
float fy = 300.0;
float cx = 300.0;
float cy = 300.0;
/**
  camera intrisic
  | 300  0  300|
  |  0  300 300|
  |  0   0   1 |
*/

/**
 * @brief Find the corresponding pixel in source image .
 *
 * Set Pixel Value
 * 0 < θ < π, -π/4 < ø < 7π/4
 * 0 < theta< π, -π/4 < fai < 7π/4
 *
 *
   x = -r cos θ sin ø
   y = -r sin θ
   z =  r cos θ cos ø

   tan(ø)  = - x/z;
   tan(θ)  =  ysinø/x
   we can assume r = 1 (r is relevant to the image size )
 */
void searchPixel(Mat &imgSrc ,Mat &target,int u,int v)
{

//    double start_time  = cv::getTickCount();

    float x = (u - cx)/fx;
    float y = (v-cy)/fy;
    float z = 1;
    float theta;
    float fai = atan(-x/z);  //we assume the  -π/4 < ø < π/4
    if(x==0)
    {
      theta  =atan(-y/z) ;
//        theta  = 0;
    }
    else
    {

      theta = atan(y*sin(fai)/x);
    }

    int rows = (-theta+PI/2)*srcRows/PI;
    int cols =  srcCols/(2*PI)*(-fai+PI/4.0*5);

    if(rows<0||rows>= srcRows) return;
    if(cols<0||cols>= srcCols) return;

    for (int k = 0 ;k<3;k++)
    {
       target.ptr<uchar>(v)[u*3+k]  = imgSrc.ptr<uchar>(rows)[cols*3+k];
    }
//    double duration_ms = (double(cv::getTickCount())-start_time)*1000/cv::getTickFrequency();
//          cout<< " calculate one pixel time cost : "<<duration_ms <<"ms "<<endl;

}
/**
*     @brief set the corresponding pixel in the target image .
*      x = -r cos θ sin ø
*      y = -r sin θ
*      z =  r cos θ cos ø
*      assume  r = 1
*/
void setPixel(Mat &imgSrc ,Mat &target,float theta,float fai)
{


      float x =  - cos(theta)*sin(fai);
      float y =  - sin(theta);
      float z =  cos(theta)*cos(fai);

      int u = int( x*fx/z+cx);
      int v = int( y*fx/z+cy);

      if(u>=target.cols||u<0) return ;
      if(v>=target.rows||v<0) return ;
      int rows = (-theta+PI/2)*srcRows/PI;
      int cols =  srcCols/(2*PI)*(-fai+PI/4);
      for (int k = 0;k<3;k++)
      {
         target.ptr<uchar>(v)[u*3+k]  = imgSrc.ptr<uchar>(rows)[cols*3+k];
      }

}



int main(int argc ,char **argv)
{

//    if(argc!=2 )
//    {
//        cerr<<"args error "<<endl;
//        cerr<<"usage rosrun panorama2cubemap panorama2cubemap_node image_name "<<endl;
//    }

    for (int index = 0;index <2800;index++)
    {
       char filename[100];
       sprintf(filename,"./frames/rgb%d.png",index+1);
       Mat imgSrc  =  imread(filename);
//    Mat imgSrc  = imread(argv[1]);
       if(imgSrc.empty())
      {
       cerr<<"read image error "<<endl;
       return -1;
       }
     else
     {
        srcRows = imgSrc.rows;
        srcCols = imgSrc.cols;
        cout<<"srcCols = "<<srcCols<<endl;
        cout<<"srcRows = "<<srcRows<<endl;
     }
   //convert image

//    Mat copy  (imgSrc.rows,imgSrc.cols,CV_8UC3);
    Mat cubeFront (600,600,CV_8UC3);

//     for (float theta = -PI/4;theta<PI/4;theta+=0.003)
//     {
//         for (float fai = -PI/2;fai<PI/2;fai+=0.006)
//         {
//             setPixel(imgSrc,cubeFront,theta,fai);
//         }
//     }



    for(int i = 0;i<cubeFront.rows;i++)
    {
        for(int j = 0;j<cubeFront.cols;j++)
        {
            searchPixel(imgSrc,cubeFront,j,i);
        }
    }

   imshow("front",cubeFront);
   waitKey(30);
   sprintf(filename,"./cubemap/cube%d.png",index+1);
   imwrite(filename,cubeFront);
    }

   return 0;

}



