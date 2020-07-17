**curve_fit.cpp**  
circle_fit.cpp  
ba_demo.cpp means "Bundle Adjustment"  
bal_demo.cpp means "Bundle Adjustment in the Large"  
sba_demo.cpp means "Stereo Bundle Adjustmen"  

only edge has computeError() method, since only edge has measurements, so only edge has Jacobian

data_fitting examples use only unary edges, not even binary edge, very straightforward, see curve_fit and circle_fit  

**Most of the blogs on Chinese Forum only cover main function, but not the definition of vertex or edge, especially the Jacobian part, which is very frustrating**

###  1. ICP demo (non-G2O)
this [link](https://blog.csdn.net/qq_37394634/article/details/104430716?ops_request_misc=&request_id=&biz_id=102&utm_term=linearizeOplus()&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-104430716) is an example for icp, but is different from the GICP demo in g2o examples  
Nevertheless it provides good introduction to the methods need to be overwriten for edge and vertex, see below:  

自定义顶点需要覆盖如下方法：  
virtual bool read(std::istream& is);  
virtual bool write(std::ostream& os) const;  
virtual void oplusImpl(const number_t* update);  
virtual void setToOriginImpl();

自定义边如要覆盖如下方法：  
virtual bool read(std::istream& is);  
virtual bool write(std::ostream& os) const;  
virtual void computeError();  
virtual void linearizeOplus();
## g2o GICP
please refer to teh types_icp.xx and gicp_demp.cpp for more information  
this example is very weird, point is not vertex, only two vertices are cam poses, all the points are edges connecting two poses  
what is dR1dx ?

### 2. Bundle Adjustment demo
the ba_demo.cpp in g2o examples is used to estimate the camera poses and feature points, not the camera parameters, the edage and vertex are defined in type_six_dof_expmap.XX please refer to the GAOXIANG book and this [link](https://blog.csdn.net/qq_25458977/article/details/100170663) for the Jacobian derivation XYZ->SE3

see also [this](https://blog.csdn.net/heyijia0327/article/details/51773578) or [this](https://blog.csdn.net/a356337092/article/details/83549298) for SE3->SE3 Jacobian derivation


### 3. Bundle Adjustment for the Large, see this [link](http://grail.cs.washington.edu/projects/bal/) for detail and dataset
This bal_demo is very different from ba_demo or sba_demo, since this demo optimize the camera intrinsic parameters instead of camera poses or feature positions


**type_icp.xx**   
Edge_V_V_GICP  
**types_six_dof_expmap.xx**    
EdgeSE3Expmap(see [link1](https://blog.csdn.net/heyijia0327/article/details/51773578) for Jacobian derivation, I have also derived.

EdgeProjectXYZ2UV(see gaoxiang for Jacobian derivation)  

Edge_V_V_GICP and EdgeSE3Expmap both connect two SE3, but the errors are different,  
For Edge_V_V_GICP, error is euclidean distance between two 3D positions, so delta_xyz/delta_SE3 Jacobian, use GAOXIANG slambook
For EdgeSE3Expmap, error is itself lie group SE3, do delta_SE3/delta_SE3 Jacobian, use ethan script with link1  

When use VertexSE3Expmap please note that the setEstimate()'s argument should be anti-pose, if you want to directly use g2o's definitions of edges  

In G2O definition of SE3, Rotation precedes Translation, as is in Maani Slide 
while in GAOXIANG Slambook, Ethan Handbook, Translation precedes Rotation  
Need to pay special attention to the order, when calculate Jacobian  

bal_example.cpp bundle adjustment at large is using **ceres::internal::AutoDifferentiate** for Jacobian Calculation, **NOT** user provided.