# 代码修改

-----
记录ORBSLAM2中自己修改的部分：
* Frame :   
``UndistortionPanomaric()``  
``ComputeImageBounds()``


* Tracking   
``GrabImageMonocular()``  
``Track()``  
``MonocularInitialization()``  
> mvbPreMatch保存的是可能匹配上的点坐标
> mvIniMatches 记录那些点是匹配上的 初始化赋值为-1



* ORBextractor   
``ORBextractor()``

* Initializer  
``Initializer(const Frame &ReferenceFrame, float sigma,int iterations)``  
> sigma 默认使用1 ，MaxIteration默认为200  

 ``Initialize()``  
 ``FindFundamental()``  
 ``CheckFundamental()``  
 

* Conventer    
``toPinholePoint2f ``  
