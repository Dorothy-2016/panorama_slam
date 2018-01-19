# 代码修改
在修改程序之前我们需要是分明确什么地方是要修改的，主要是正确panoramic camera的相机模型进行修改：  
* 1.initialization  
* 2.Bundle adjustment  
* 3.Tracking

-----
### 记录ORBSLAM2中自己修改的部分：
## Frame :   
``UndistortionPanomaric()``  
``ComputeImageBounds()``  
``isInFrustum()``  



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

  #### 1. TrackReferenceKeyFrame()
  	_step 1. mCurrentFrame  computeBow()_  
    _step 2. match  SearchByBow() match是在当前帧跟关键帧对应的地图点之间的 ,然后将匹配的结果（MapPoint* )赋给mCurrentFrame.mvpMapPoints_   
    _step 3.  PoseOptimization(mCurrentFrame)_ 
 #### 2. TrackWithMotionModel()  
    _step 1. updateLastFrame() _  //单目情况下直接退出  
    _step 2. SearchByprojection()_  
    _step 3. PoseOptimization()_  
  #### 3. TrackLocalMap()
    _step 1. UpdateLocalMap()_  
      $\space \rightarrow$ UpdateLocalKeyFrames()  
      $\space \rightarrow$  UpdateLocalPoints()  
    _step 2. SearchLocalPoints()_   
    _step 3. PoseOptimization()_ 
    
    ###### 总结三种Track模式都是要找到对应的MapPoints
 
 
  

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
``LocalBundleAdjustment()``  


## ORBmatcher
``SearchByProjection(Frame,Frame,float,bool)``   
``SearchByProjection(Frame,vector<MapPoint*>,float)``   
``SearchByProjection(Frame,KyeFrame,set<MapPoint*>,float,int)``   
``SearchForTriangulation()``   其中对于ex,ey与keypoint位置判断特征点距离的方法不妥当  
``checkFundamental()``  


## LocalMapping 
``ProcessNewKeyFrame()``  _好像也没什么好改的  
``CreateNewMapPoints()``   非常复杂
``ComputeF12()``   set K identity  这里面有一个特别麻烦的东西 searchForTriangulation





## 一些细节  
* 在Fuse 和 SearchAndFuse函数会对MapPoint进行融合 在track之前需要进行判断
* 在tracking过程中会使用使用UpdateLastFrame 对当前帧临时添加一些MapPoint进行跟踪，在跟踪结束后需要对其进行删除 
* 在tracking中 调用BundleAdjustment	会返回mvbOutlier  
* 在trackLocalMap中会将使用searchLocalPoints()将mvpLocalMapPoints全部投影到当前帧，坐标在MapPoint中的mTrackProjX and mTrackProjY
* 有一个疑问，在PoseOptimization()中，总共会进行四次迭代，但是在迭代过程中为什么不是将上一次的结果作为下一次迭代的初始值？
* 接下来我们需要使用全部图片    （现在出现的问题是 LocalMapping里应该已经成功生成的了地图点,但是在某处把这些点判断成badpoint又删除了)
$\longrightarrow$修改在各处判断深度值必须为正数的地方  
		1.ORBmatcher::Fuse 			if(p3Dc.at<float>(2)<0.0f)
		2.ORBmatcher::SearchByProjection(Frame ,Frame,..)  
		3.Frame::isInFrustum()
修改了Tracking中 initORBextrator features的数量 
