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
