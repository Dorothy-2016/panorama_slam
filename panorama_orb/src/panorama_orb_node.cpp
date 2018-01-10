#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iterator>
#include <iostream>
#include <thread>
#include "ORBextractor.h"
#include "Thirdparty/DBoW2/DUtils/Random.h"


#define PI 3.1415926

using namespace cv;
using namespace std;
float fx = 300.0;
float fy = 300.0;
float cx = 300.0;
float cy = 300.0;
//image resolution 1920 960
//x = -r cos θ sin ø
//y = -r sin θ
//z =  r cos θ cos ø
/**
 * @brief DrawORBKeyPoints
 * @param image
 * @param mvKeys
 */
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


void test(cv::Mat srcImage)
{


    Mat mask = Mat::zeros(srcImage.size(), CV_8U);  // type of mask is CV_8U
    Mat roi(mask, cv::Rect(300,300,500,200));

    roi = Scalar(255, 255, 255);
    imwrite("mask_roi.png",mask);
    cout<<mask.type()<<endl;
    Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(20);
    std::vector<KeyPoint> keypoints;
    detector->detect(srcImage, keypoints, mask);

    Mat img_keypoints;
    drawKeypoints(srcImage, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
        //-- 显示特征点
        imshow("Keypoints", img_keypoints);
        imwrite("fast_detection_result.png", img_keypoints);

        waitKey(0);
        return ;
}
/**
 * @brief CreateMask
 * @param Height
 * @param Width
 * @return  Mask Mat
 */
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
/**
 * @brief CreateMaskUseLess  create a white mask (means the mask is useless )
 * @param Height
 * @param Width
 * @return  Mask Mat
 */
cv::Mat CreateMaskUseLess(int Height, int Width)
{

    Mat tempMask = Mat::zeros(Height,Width, CV_8U);
    for (int i  = 0 ;i<Height;i++)
      {
        for (int j = 0;j<Width;j++)
        {
             tempMask.at<uchar>(i, j)= 255;

        }
    }
    return tempMask;
}

void  OpenCV3BuildInORB(char* imagename1,char* imagename2)
{

    /**

      ORB Feature match

     **/
             Mat rgbd1 = imread(imagename1);
             Mat rgbd2= imread(imagename2);

             Ptr<ORB> orb = ORB::create();
             vector<KeyPoint> Keypoints1,Keypoints2;
             Mat descriptors1,descriptors2;
             orb->detectAndCompute(rgbd1, Mat(), Keypoints1, descriptors1);
             orb->detectAndCompute(rgbd1, Mat(), Keypoints2, descriptors2);

             //cout << "Key points of image" << Keypoints.size() << endl;

             //可视化，显示关键点
             Mat ShowKeypoints1, ShowKeypoints2;
             drawKeypoints(rgbd1,Keypoints1,ShowKeypoints1);
             drawKeypoints(rgbd2, Keypoints2, ShowKeypoints2);
             cout<<"keypoints : "<<Keypoints1.size()<<endl;
             cout<<"keypoints : "<<Keypoints2.size()<<endl;
             for (int i = 0 ;i<Keypoints1.size();i++)
             {

                 cout<<Keypoints1[i].octave<<endl;
             }

             imshow("Keypoints1", ShowKeypoints1);
             imshow("Keypoints2", ShowKeypoints2);

             waitKey(0);

             //Matching
             vector<DMatch> matches;
             Ptr<DescriptorMatcher> matcher =DescriptorMatcher::create("BruteForce");
             matcher->match(descriptors1, descriptors2, matches);
             cout << "find out total " << matches.size() << " matches" << endl;

             //可视化
             Mat ShowMatches;
             drawMatches(rgbd1,Keypoints1,rgbd2,Keypoints2,matches,ShowMatches);
             imshow("matches", ShowMatches);

             imwrite("matches.png",ShowMatches);
             waitKey(0);

}
/**
 * @brief FindEssential
 * @param mvKeys1   Query Keypoints
 * @param mvKeys2   Train Keypoints
 * @param matches   Good Matches
 */


void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    meanX = meanX/N;
    meanY = meanY/N;

    float meanDevX = 0;
    float meanDevY = 0;

    // 将所有vKeys点减去中心坐标，使x坐标和y坐标均值分别为0
    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;

    float sX = 1.0/meanDevX;
    float sY = 1.0/meanDevY;

    // 将x坐标和y坐标分别进行尺度缩放，使得x坐标和y坐标的一阶绝对矩分别为1
    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    // |sX  0  -meanx*sX|
    // |0   sY -meany*sY|
    // |0   0      1    |
    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}

void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);

    // 对 t 有归一化，但是这个地方并没有决定单目整个SLAM过程的尺度
    // 因为CreateInitialMapMonocular函数对3D点深度会缩放，然后反过来对 t 有改变
    u.col(2).copyTo(t);
    t=t/cv::norm(t);

    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;

    R1 = u*W*vt;
    if(cv::determinant(R1)<0) // 旋转矩阵有行列式为1的约束
        R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
        R2=-R2;
}
cv::Mat ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(N,9,CV_32F); // N*9

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3); // v的最后一列

    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2)=0; // 秩2约束，将第3个奇异值设为0

    return  u*cv::Mat::diag(w)*vt;
}
void Convert2PinholePoint(cv::KeyPoint kp,float &x ,float &y)
{
    float coefx = 2*PI/1920.0;
    float coefy = PI/960.0;

    float px = kp.pt.x ;
    float py = kp.pt.y ;
    float theta = -px*coefx+3.0/2.0*PI;
    float fai = -py*coefy+PI/2.0;
    x = 1.0/tan(theta);
    y = -tan(fai)/sin(theta);
    //z = 1.0;
}
float CheckEssential(std::vector<cv::KeyPoint>  vPKeys1,std::vector<cv::KeyPoint>vPKeys2,std::vector<cv::DMatch> matches ,cv::Mat E)
{
    //compute the 3d
    std::vector<cv::KeyPoint>  keys1 ,keys2;

    for (int i = 0;i<vPKeys1.size();i++)
    {   //first point

        float x ,y ;
        Convert2PinholePoint(vPKeys1[i],x,y);
        cv::KeyPoint newkp;
        newkp.pt.x = x;
        newkp.pt.y = y;
        keys1.push_back(newkp);
    }
    for (int i = 0;i<vPKeys2.size();i++)
    {   //first point

        float x ,y ;
        Convert2PinholePoint(vPKeys2[i],x,y);
        cv::KeyPoint newkp;
        newkp.pt.x = x;
        newkp.pt.y = y;
        keys2.push_back(newkp);
    }
    int counter =0;
    float sum = 0;
    for (int i =0 ;i<matches.size();i++)
    {
        int queryId = matches[i].queryIdx;
        int trainId = matches[i].trainIdx;
        cv::KeyPoint kp1 = keys1[queryId];
        cv::KeyPoint kp2 = keys2[trainId];
        cv::Mat mkp1(1,3 ,CV_32F);
        cv::Mat mkp2(3,1 ,CV_32F);
        mkp1.at<float>(0,0) = (float)kp1.pt.x;
        mkp1.at<float>(0,1) = (float)kp1.pt.y;
        mkp1.at<float>(0,2) = 1.0f;

        mkp2.at<float>(0,0) = (float)kp2.pt.x;
        mkp2.at<float>(1,0) = (float)kp2.pt.y;
        mkp2.at<float>(2,0) = 1.0f;

        Mat res = mkp1*E*mkp2 ;
        if(res.at<float>(0,0)>10.0) continue;
        counter++;
        sum += res.at<float>(0,0);
    }
    cout<<sum/counter<<endl;
    return sum/counter;

}
cv::Mat FindEssential(std::vector<cv::KeyPoint> vPKeys1,std::vector<cv::KeyPoint>vPKeys2,std::vector<cv::DMatch> matches,float &res)
{
    int nmatches  = matches.size();
//    cout<<"nmatches : "<<nmatches<<endl;
    cout<<" random int ";
    if(nmatches <= 8) return cv::Mat() ;
    vector<int> vEightpoint;
    for (int i = 0 ;i<8;i++)
    {
     DUtils::Random::SeedRandOnce(0);
     int randi = DUtils::Random::RandomInt(0,nmatches-1);
     vEightpoint.push_back(randi);
     cout<<randi<<" ";
//     cout<<" random int "<<randi<<endl;
    }
    cout<<endl;


    vector<cv::KeyPoint> mvKeys1,mvKeys2;
    cv::KeyPoint newkp ;
    float coefx = 2*PI/1920.0;
    float coefy = PI/960.0;
//    cout<<"nmatches : "<<nmatches<<endl;

    //Change to pinhole point
    for(int i = 0;i<8;i++)
    {   int index = vEightpoint[i];
        int queryid  = matches[index].queryIdx;
        cv::KeyPoint  kp1 = vPKeys1[queryid];
        float x = kp1.pt.x ;
        float y = kp1.pt.y ;
        float theta = -x*coefx+3.0/2.0*PI;
        float fai = -y*coefy+PI/2.0;
        newkp.pt.x = 1.0/tan(theta);
        newkp.pt.y = -tan(fai)/sin(theta);
        mvKeys1.push_back(newkp);

    }

    for (int i = 0;i<8;i++)
    {
        int index = vEightpoint[i];
        int trainid  = matches[index].trainIdx;
        cv::KeyPoint  kp2 = vPKeys2[trainid];
        float x = kp2.pt.x ;
        float y = kp2.pt.y ;
        float theta = -x*coefx+3.0/2.0*PI;
        float fai = -y*coefy+PI/2.0;
        newkp.pt.x = 1.0/tan(theta);
        newkp.pt.y = -tan(fai)/sin(theta);
        mvKeys2.push_back(newkp);
    }

//    cout<<"mvkeys  : "<< mvKeys1.size() <<" "<<mvKeys2.size()<<endl;


    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2t = T2.t();

    // Iteration variables
//    vector<cv::Point2f> vPn1i(8);
//    vector<cv::Point2f> vPn2i(8);
    cv::Mat F21i;
    cv::Mat Fn = ComputeF21(vPn1,vPn2);
    F21i = T2t*Fn*T1;
    cv::Mat E21 = F21i;

//    cout<<"R1"<<endl;
//    cout<<R1<<endl;
//    cout<<"R2"<<endl;
//    cout<<R2<<endl;
//    cout<<"t1"<<endl;
//    cout<<t1<<endl;
//    cout<<"t2"<<endl;
//    cout<<t2<<endl;

    res = CheckEssential(vPKeys1,vPKeys2,matches,E21);
    return E21;

}


/**
 * @brief main  Code entrance
 * @param argc
 * @param argv
 * @return
 */

int main(int argc,char ** argv)
{
  if(argc!=3)
  {
      cerr<<"arg error  :"<<endl;

  }

      int nfeatures =  1000;
      int level = 8;
      cout<<"nfeatures : "<<nfeatures<<endl;
      cout<<"level : "<<level<<endl;


  Mat image = imread(argv[1]);
  Mat image2 = imread(argv[2]);
  if(image.empty()||image2.empty()){
      cerr<<"image empty"<<endl;
      return -1;
  }
          if(image.channels()==3)
         {
                 cvtColor(image,image,CV_RGB2GRAY);
         }
          if(image2.channels()==3)
         {
                 cvtColor(image2,image2,CV_RGB2GRAY);
         }

      Mat Mask = CreateMask(image.rows,image.cols);
      imshow("mask",Mask);
      imwrite("mask.png",Mask);
      cv::waitKey(0);

      ORBextractor* mpORBextractor;
      mpORBextractor  = new ORBextractor(nfeatures,1.2,level,20,7);


      std::vector<cv::KeyPoint> mvKeys;
      cv::Mat mDescriptors;
      std::vector<cv::KeyPoint> mvKeys2;
      cv::Mat mDescriptors2;

      (*mpORBextractor)(image,Mask,mvKeys,mDescriptors);
      (*mpORBextractor)(image2,Mask,mvKeys2,mDescriptors2);


      DrawORBKeyPoints(image,mvKeys);
      DrawORBKeyPoints(image2,mvKeys2);

      vector<DMatch> matches,goodmatches;
      Ptr<DescriptorMatcher> matcher =DescriptorMatcher::create("BruteForce");
      matcher->match(mDescriptors, mDescriptors2, matches);
      // query  train
      cout << "find out total " << matches.size() << " matches" << endl;



      int nMinDis=100,nMaxDis=0;
      for (size_t i=0;i<matches.size();i++)
      {
          if (nMinDis>matches[i].distance)
              nMinDis=matches[i].distance;
          if (nMaxDis<matches[i].distance)
              nMaxDis=matches[i].distance;
      }
      cout<<"最大距离："<<nMaxDis<<endl;
      cout<<"最小距离："<<nMinDis<<endl;
      for (size_t i=0;i<matches.size();i++)
      {
          if (matches[i].distance < nMaxDis * 0.4)   //0.3为参考值，距离是指两个特征向量间的欧式距离，表明两个特征的差异，
                                                      //值越小表明两个特征点越接近 。越小结果越精，但剩下的点少；越大结果
                                                      //越粗，剩下的点多
              goodmatches.push_back(matches[i]);
      }
      //可视化
      Mat ShowMatches;
      drawMatches(image,mvKeys,image2,mvKeys2,goodmatches,ShowMatches);
      cout<<"good matches "<<goodmatches.size()<<endl;
      imshow("matches", ShowMatches);
      cv::waitKey(0);

      std::vector<cv::Mat> vE;
      std::vector<float> vres;
      for (int i = 0;i<10;i++)
      {
      float res =0;
      cv::Mat E = FindEssential(mvKeys,mvKeys2,goodmatches,res);
      vE.push_back(E);
      vres.push_back(res);
      }
      float minRes  = 100.0;
      cv::Mat bestE ;
      for (int i = 0;i<vE.size();i++)
      {
          if(vres[i]<minRes)
          {
              minRes = vres[i];
              bestE = vE[i];
          }
      }

      cv::Mat R1 ,R2,t1;
      DecomposeE(bestE,R1,R2,t1);
      cv::Mat t2 = -t1;

      cout<<"R1"<<endl;
      cout<<R1<<endl;
      cout<<"R2"<<endl;
      cout<<R2<<endl;
      cout<<"t1"<<endl;
      cout<<t1<<endl;
      cout<<"t2"<<endl;
      cout<<t2<<endl;





  return -1;
}
