## Panorama SLAM
------
### Essential matrix 
$  p = (u,v) $ 
$$ \theta = -u*coef_x+3/2\pi$$
$$ \phi = -v*coef_y+\pi/2$$ p is a feature point in the panoramic image  
$$x = r cos(\phi )cos(\theta ) $$
$$y = -r sin(\phi )$$
$$z = r cos(\phi)sin(\theta)$$
so the 3D point Essential Matrix : 
$$ P_1^{T}EP_2=0$$
$$r_1r_2z_1z_2\bar{P_1^{T}}EP_2=0$$
$$\bar{P_1^{T}}EP_2=0$$
Where  :$\bar{P_n} = r\begin{bmatrix}
cot(\theta_n)\\-tan(\phi_n)/sin(\theta_n)\\1
\end{bmatrix}$
$$E= \hat{t}R$$


>在分解本质矩阵的过程中会遇到 4 solutions 的问题；
常见的办法就是判断4个解各自恢复的点的深度，但是对于panoramic image 正确解也会产生负数的深度值。
solution: 
在初始化的过程中只使用前向视角的特征点，在确定了正确解之后再对后面的点进行深度值的恢复；

## Mono-feature Triangular 
$$P_1=M_1P$$
$$P_2=M_2P$$
$M_1 = I $ ,$ M_2 = [R|t]$
跟传统的深度估计不同的是，传统方法中$P_n$是图像坐标构成的，因此$M_n$中包含了相机内参。而在本应用中$P_n$是三维点，因此 $M_n$中不包含内参成分，剩下的求解过程相同，构建方程->SVD->搞定。

---
## Optimization (difficult) 
假设图像的误差函数仍然是像素之间的误差
$$e(\xi)=\sum(\begin{bmatrix}
x-x_{match}\\y-y_{match}\end{bmatrix})$$
$$J=\frac{\partial e(\xi)}{\partial \xi}$$
$$J=J_1J_2J_3$$
$$J_1 = \frac{\partial e(\xi)}{\partial {p}'}=\begin{bmatrix} 
\frac{\partial ({x}'-x_{match})}{\partial {x}'} &\frac{\partial ({x}'-x_{match})}{\partial {y}'}
\\ \frac{\partial ({y}'-y_{match})}{\partial {x}'} & 
\frac{\partial ({y}'-y_{match})}{\partial {y}'} \end{bmatrix}=
\begin{bmatrix}1 & 0 \\0 &1 \end{bmatrix}$$
$$J_2  =  \frac{\partial {p}'}{\partial P}=\frac{\partial [{x}'\space  {y}']}{\partial [X \space Y \space Z]}=\begin{bmatrix}
\frac{\partial {x}'}{\partial X}&
\frac{\partial {x}'}{\partial Y}&
\frac{\partial {x}'}{\partial Z}\\
\frac{\partial {y}'}{\partial X}&
\frac{\partial {y}'}{\partial Y}&
\frac{\partial {y}'}{\partial Z}
\end{bmatrix}$$
$$\theta = atan(\frac{Z}{X})$$
$$\phi = asin(\frac{-Y}{\sqrt{X^2+X^2+Z^2}})= -asin(\frac{Y}{\sqrt{X^2+X^2+Z^2}})$$
$${x}' = [-atan(\frac {Z}{X})+C]coefu$$
$${y}'=[asin(\frac{y}{\sqrt{x^2+y^2+z^2}})+D]coefv$$

求导公式${arctan(x)}'=\frac{1}{1+x^2} $ , 
${arcsin(x)}'=\frac{1}{\sqrt{1-x^2}}$
$$\frac{\partial {x}'}{\partial X}=coefu\frac{Z}{X^2+Z^2}$$
$$\frac{\partial {x}'}{\partial Y}=0$$
$$\frac{\partial {x}'}{\partial Z}=-coefu\frac{X}{X^2+Z^2}$$
$$\frac{\partial {y}'}{\partial X}=-coefv\frac{XY}{\sqrt{X^2+Z^2}(X^2+Y^2+Z^2)}$$
$$\frac{\partial {y}'}{\partial Y}=coefv\frac{\sqrt{X^2+Z^2}}{X^2+Y^2+Z^2}$$
$$\frac{\partial {y}'}{\partial Z}=-coefv\frac{YZ}{\sqrt{X^2+Z^2}(X^2+Y^2+Z^2)}$$
so :
$$J2 = \begin{bmatrix}
coefu\frac{Z}{X^2+Z^2} &
0& 
-coefu\frac{X}{X^2+Z^2} \\
-coefv\frac{XY}{\sqrt{X^2+Z^2}(X^2+Y^2+Z^2)} &
coefv\frac{\sqrt{X^2+Z^2}}{X^2+Y^2+Z^2} &
-coefv\frac{YZ}{\sqrt{X^2+Z^2}(X^2+Y^2+Z^2)}
\end{bmatrix}$$

$$J3= \begin{bmatrix}
1 &  0 & 0 & 0 & Z & -Y\\
0 & 1 & 0 & -Z & 0 & X\\
0 & 0 & 1 & Y & -X & 0 
\end{bmatrix}$$
综上： : )
$$J = \begin{bmatrix}
coefu\frac{Z}{X^2+Z^2} &
0& 
-coefu\frac{X}{X^2+Z^2} &
-coefu\frac{XY}{X^2+Z^2}&
-coefu&
-coefu\frac{YZ}{X^2+Z^2}\\
-coefv\frac{XY}{\sqrt{X^2+Z^2}(X^2+Y^2+Z^2)} &
coefv\frac{\sqrt{X^2+Z^2}}{X^2+Y^2+Z^2} &
-coefv\frac{YZ}{\sqrt{X^2+Z^2}(X^2+Y^2+Z^2)}&
-coefv\frac{Z}{\sqrt{X^2+Z^2}}&
0&
coefv\frac{X}{\sqrt{X^2+Z^2}}
\end{bmatrix}
$$

$$$$

----
## ORB_SLAM2 Optimizer.cpp 
#### EdgeSE3ProjectXYZ
##### 姿态 VetexSE3Expmap 

  * $ T \leftarrow exp(\hat{\xi})T \space  $  使用的是左乘法更新 
  * $ \xi $ = [w1 w2 w3 v1 v2 v3]  
   >  virtual void oplusImpl(const double* update_)  { </br>
  Eigen::Map<const Vector6d> update(update_);  
    setEstimate(SE3Quat::exp(update)*estimate());
  } 

##### 误差边 EdgeSE3ProjectXYZ
  * 重投影误差
> void computeError()  {  </br>
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]); // T</br>
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]); // Xw</br>
    Vector2d obs(_measurement);</br>
    _error = obs-cam_project(v1->estimate().map(v2->estimate()));</br>
  }
  
  * 误差函数对[X Y Z]和增量[w1 w2 w3 v1 v2 v3]求雅克比矩阵 
   * $Ji_{2\times3} = \frac{\partial [obs - \Pi(exp(\hat{\xi}) T X_w)]}{\partial X_w} $ 
   * $Jj_{2\times6} = \frac{\partial [obs - \Pi(exp(\hat{\xi}) T X_w)]}{\partial \xi} $ 
   * @note _jacobianOplusXi，_jacobianOplusXj
  




