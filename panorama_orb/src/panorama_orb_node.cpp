#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iterator>
#include <iostream>
#include <fstream>
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
           if(j<500||j>1420) continue;
           int pixcor = j%(Width/4) -Width/8;
           float fai = PI/4-fabs( float(pixcor)/Width*2*PI);
           float theta =  float(-i+Height/2)/Height*PI;

           float x = -cos(theta)*sin(fai);
           float y = -sin(theta);
           float z =  cos(theta)*cos(fai);
           float u = (x/z*fx+cx) ;
           float v = (y/z*fy+cy) ;

         // tempMask.at<uchar>(i, j)= 255;
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

void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.pt.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.pt.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.pt.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.pt.y*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
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
float CheckEssential(std::vector<cv::KeyPoint>  vPKeys1,std::vector<cv::KeyPoint>vPKeys2,std::vector<cv::DMatch> &matches ,cv::Mat E)
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



//        Mat res = mkp2.t()*E*mkp1.t();
        if(fabs(res.at<float>(0,0))>0.10)   //the error is too big  remove
        {
             matches[i] = matches.back();
             matches.pop_back();
             continue;
        }
        counter++;
        sum += fabs(res.at<float>(0,0));
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


int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vPKeys1, const vector<cv::KeyPoint> &vPKeys2,
                       const vector<cv::DMatch> &vMatches12, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
//    // Calibration parameters
//    const float fx = K.at<float>(0,0);
//    const float fy = K.at<float>(1,1);
//    const float cx = K.at<float>(0,2);
//    const float cy = K.at<float>(1,2);
    std::vector<cv::KeyPoint> vKeys1,vKeys2;
    for (int i = 0 ;i< vPKeys1.size();i++)
    {
        float x ,y ;
        Convert2PinholePoint(vPKeys1[i],x,y);
        cv::KeyPoint newKp1;
        newKp1.pt.x = x;newKp1.pt.y = y;
        vKeys1.push_back(newKp1);

        Convert2PinholePoint(vPKeys2[i],x,y);
        cv::KeyPoint newKp2;
        newKp2.pt.x = x;newKp2.pt.y = y;
        vKeys2.push_back(newKp2);

    }

    vbGood = vector<bool>(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    cv::Mat K  =cv::Mat::eye(3,3,CV_32F);
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));

    // 第一个相机的光心在世界坐标系下的坐标
    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);


    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;
    // 第二个相机的光心在世界坐标系下的坐标
    cv::Mat O2 = -R.t()*t;

    int nGood=0;

    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
               // kp1和kp2是匹配特征点
        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].queryIdx];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].trainIdx];
        cv::Mat p3dC1;

        // 步骤3：利用三角法恢复三维点p3dC1
        Triangulate(kp1,kp2,P1,P2,p3dC1);

        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].queryIdx]=false;
            continue;
        }

        // Check parallax
        // 步骤4：计算视差角余弦值
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);

        float cosParallax = normal1.dot(normal2)/(dist1*dist2);

        // 步骤5：判断3D点是否在两个摄像头前方

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        // 步骤5.1：3D点深度为负，在第一个摄像头后方，淘汰
        if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        // 步骤5.2：3D点深度为负，在第二个摄像头后方，淘汰
        cv::Mat p3dC2 = R*p3dC1+t;

        if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // 步骤6：计算重投影误差

        // Check reprojection error in first image
        // 计算3D点在第一个图像上的投影误差
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        im1x = p3dC1.at<float>(0)*invZ1;
        im1y = p3dC1.at<float>(1)*invZ1;

        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        // 步骤6.1：重投影误差太大，跳过淘汰
        // 一般视差角比较小时重投影误差比较大
        if(squareError1>th2)
            continue;

        // Check reprojection error in second image
        // 计算3D点在第二个图像上的投影误差
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = p3dC2.at<float>(0)*invZ2;
        im2y = p3dC2.at<float>(1)*invZ2;

        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        // 步骤6.2：重投影误差太大，跳过淘汰
        // 一般视差角比较小时重投影误差比较大
        if(squareError2>th2)
            continue;

        // 步骤7：统计经过检验的3D点个数，记录3D点视差角
        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].queryIdx] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;

        if(cosParallax<0.99998)
            vbGood[vMatches12[i].queryIdx]=true;
    }

    cout<<"vcosparallax size : "<<vCosParallax.size()<<endl;
    // 步骤8：得到3D点中较大的视差角
    if(nGood>0)
    {
        // 从小到大排序
        sort(vCosParallax.begin(),vCosParallax.end());

        // trick! 排序后并没有取最大的视差角
        // 取一个较大的视差角
        size_t idx = min(20,int(vCosParallax.size()-1));
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
        parallax=0;

    return nGood;
}


void CreateInitialMap(std::vector<cv::Point3f> p3d,std::vector<bool>vbTriangular,
                      std::vector<cv::KeyPoint> mvKeys1 ,std::vector<cv::KeyPoint> mvKeys2,std::vector<cv::DMatch> matches)
{
   cout<<"p3d size "<< p3d.size()<<endl;
   cout<<"vbTriangular"<<vbTriangular.size()<<endl;
   ofstream  fout("mappoint.txt");
   ofstream  corfout("keypoints.txt");
   int count =0;
   for (int i = 0 ;i<p3d.size();i++)
   {
       if(vbTriangular[i])
       {
           fout<<p3d[i].x<<" "<<p3d[i].y<<" "<<p3d[i].z<<endl;
           count ++ ;
           int queryIdx = i ;
           for (int j = 0 ;j<matches.size();j++)
           {
              if(queryIdx == matches[i].queryIdx)
              {
              int trainIdx  = matches[i].trainIdx;
              cv::Point2f kp1 =  mvKeys1[queryIdx].pt;
              cv::Point2f kp2 =  mvKeys2[trainIdx].pt;
               corfout<< kp1.x <<" "<<kp1.y<<" "<<kp2.x<<" "<<kp2.y<<endl;
               break;
              }

           }
       }
   }
   fout.close();
   corfout.close();
   if(count<=50)
   {
       cout<<"create Initial Map error ,too few Map point"<<endl;
       cout<<"#############################################"<<endl;
   }
   else
   {

       cout<<"create "<<count<<" map points"<<endl;
   }


}

void SaveRT(cv::Mat R,cv::Mat t)
{
    ofstream fout("pose.txt");
    cout<<"save R t"<<endl;
    cout<<"R"<<endl<<R<<endl;
    cout<<"t"<<endl<<t<<endl;
    for (int i =  0;i<3;i++)
    {
        for (int j = 0;j<3;j++)
        {
            fout<< R.at<float>(i,j)<<" ";

        }
        fout<<endl;
    }

    for (int i = 0;i<3;i++)
    {
        fout<<t.at<float>(i,0)<<" ";
    }

    fout.close();

}

/**
 * @brief main  Code entrance
 * @param argc
 * @param argv
 * @return
 */

int main(int argc,char ** argv)
{
  if(argc!=2)
  {
      cerr<<"arg error  :"<<endl;

  }

      int nfeatures =  2000;
      int level = 8;
      cout<<"nfeatures : "<<nfeatures<<endl;
      cout<<"level : "<<level<<endl;

  int frameIdx = atoi(argv[1]);
  char filename[100];
  sprintf(filename,"frames/rgb%d.png",frameIdx);
  frameIdx = frameIdx+5;
  Mat image = imread(filename);
  sprintf(filename,"frames/rgb%d.png",frameIdx);
  Mat image2 = imread(filename);
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

      (*mpORBextractor)(image,cv::Mat(),mvKeys,mDescriptors);
      (*mpORBextractor)(image2,cv::Mat(),mvKeys2,mDescriptors2);


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
          if (matches[i].distance < nMaxDis * 0.35)   //0.3为参考值，距离是指两个特征向量间的欧式距离，表明两个特征的差异，
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
      std::vector<std::vector<cv::DMatch> > vmatches;

      for (int i = 0;i<200;i++)
      {
      float res =0;
      std::vector<DMatch>  currentMatches ;
      currentMatches  = goodmatches;
      cv::Mat E = FindEssential(mvKeys,mvKeys2,currentMatches,res);
      vE.push_back(E);
      vres.push_back(res);
      vmatches.push_back(currentMatches);
      }
      float minRes  = 100.0;
      cv::Mat bestE ;
      std::vector<DMatch> bestMatches ;
      for (int i = 0;i<vE.size();i++)
      {
          if(vres[i]<minRes)
          {
              minRes = vres[i];
              bestE = vE[i];
              bestMatches  =  vmatches[i];
          }
      }

      cv::Mat R1 ,R2,t1;
      DecomposeE(bestE,R1,R2,t1);
      t1  = t1/cv::norm(t1)*0.01;
      cv::Mat t2 = -t1;

      cout<<"R1"<<endl;
      cout<<R1<<endl;
      cout<<"R2"<<endl;
      cout<<R2<<endl;
      cout<<"t1"<<endl;
      cout<<t1<<endl;
      cout<<"t2"<<endl;
      cout<<t2<<endl;

      vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
      vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
      float parallax1=0.0,parallax2=0.0, parallax3=0.0, parallax4=0.0;
      int Good1,Good2,Good3,Good4;
      Good1 = CheckRT(R1,t1,mvKeys,mvKeys2,bestMatches,vP3D1,0.04,vbTriangulated1,parallax1);
      Good2 = CheckRT(R1,t2,mvKeys,mvKeys2,bestMatches,vP3D2,0.04,vbTriangulated2,parallax2);
      Good3 = CheckRT(R2,t1,mvKeys,mvKeys2,bestMatches,vP3D3,0.04,vbTriangulated3,parallax3);
      Good4 = CheckRT(R2,t2,mvKeys,mvKeys2,bestMatches,vP3D4,0.04,vbTriangulated4,parallax4);
      cout <<"R1 t1 "<<Good1<<endl;
      cout <<"R1 t2 "<<Good2<<endl;
      cout <<"R2 t1 "<<Good3<<endl;
      cout <<"R2 t2 "<<Good4<<endl;


      cout<<parallax1<<endl;
      cout<<parallax2<<endl;
      cout<<parallax3<<endl;
      cout<<parallax4<<endl;


       int nMaxGood  = 0 ;
       if(Good1>nMaxGood) nMaxGood = Good1;
       if(Good2>nMaxGood) nMaxGood = Good2;
       if(Good3>nMaxGood) nMaxGood = Good3;
       if(Good4>nMaxGood) nMaxGood = Good4;

       int threshold = static_cast<int>(0.85*bestMatches.size());
       cout<<"threshold : "<<threshold<<endl;
       cout<<" nMaxGood "<<nMaxGood<<endl;

       if(nMaxGood<threshold)
       {
        cout<<"initial failed"<<endl;
        cout<<"#############################################"<<endl;
        return -1;
       }

        if(Good1==nMaxGood)
        {
            CreateInitialMap(vP3D1,vbTriangulated1,mvKeys,mvKeys2,matches);
            SaveRT(R1,t1);
        }
        if(Good2==nMaxGood)
        {
            CreateInitialMap(vP3D2,vbTriangulated2,mvKeys,mvKeys2,matches);
            SaveRT(R1,t2);
        }
        if(Good3==nMaxGood)
        {
            CreateInitialMap(vP3D3,vbTriangulated3,mvKeys,mvKeys2,matches);
            SaveRT(R2,t1);
        }
        if(Good4==nMaxGood)
        {
            CreateInitialMap(vP3D4,vbTriangulated4,mvKeys,mvKeys2,matches);
            SaveRT(R2,t2);
        }




  return -1;
}
