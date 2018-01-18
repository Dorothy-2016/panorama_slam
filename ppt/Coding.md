# 代码修改
在修改程序之前我们需要是分明确什么地方是要修改的，主要是正确panoramic camera的相机模型进行修改：
1.initialization  
2.Bundle adjustment  
3.Tracking

-----
记录ORBSLAM2中自己修改的部分：
## Frame :   
``UndistortionPanomaric()``  
``ComputeImageBounds()``


## Tracking   
``GrabImageMonocular()``  
``Track()``  
``MonocularInitialization()``  
> mvbPreMatch保存的是可能匹配上的点坐标
> mvIniMatches 记录那些点是匹配上的 初始化赋值为-1   

``CreateInitialMapMonucular()``  
trakcking线程中需要用到的量  
  * mCurrentFrame $\longrightarrow $  tracking all 
  * mnLastKeyFrameId $\longrightarrow $  tracking referenceKF
  * mpLastKeyFrame    $\longrightarrow $  tracking referenceKF
  * mvpLocalKeyFrames   $\longrightarrow $  tracking localmap & referenceKF??
  * mvplocalMapPoints  $\longrightarrow $ tracking localmap
  * mpReferenceKF  $\longrightarrow $   tracking referenceKF
  * mLastFrame  $\longrightarrow $   tracking motion mode
  

## ORBextractor   
``ORBextractor()``

## Initializer  
``Initializer(const Frame &ReferenceFrame, float sigma,int iterations)``  
> sigma 默认使用1 ，MaxIteration默认为200  

 ``Initialize()``  
 ``FindFundamental()``  
 ``CheckFundamental()``    
 ``CheckFundamental_Panoramic()``
 >函数中的threshold和inlier的选择方式有待提高  

 ``ReconstructF()``   
 ``CheckRT_Panoramic()``   
 ``ReconstructF()``  
 
## Conventer    
``toPinholePoint2f ``  
## Optimizer
``BundleAdjustment()``  
``PoseOptimization()``  


## 一些细节  
* 在Fuse 和 SearchAndFuse函数会对MapPoint进行融合 在track之前需要进行判断
* 在tracking过程中会使用使用UpdateLastFrame 对当前帧临时添加一些MapPoint进行跟踪，在跟踪结束后需要对其进行删除 
* 在tracking中 调用BundleAdjustment	会返回mvbOutlier
