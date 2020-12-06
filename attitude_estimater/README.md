# pose_estimate
 
### V1.1 修改日志
	1. 修改了 Mahony AHRS 接口函数，当前问题是加入了磁力计以后滤波器不稳定，在平面上绕Z轴转动时候会导致roll和pitch角度产生几度变化,这个现象在EKF中也存在
 
### V1.0 修改日志

  增加了Mahony互补滤波算法   Mahony_AHRS.cpp  Mahony_AHRS.
  
  增加了Madgwick融合算法    Madgwick_AHRS.cpp  Madgwick_AHRS.h
  
  增加了EKF融合算法    EKF_AHRS.cpp  EKF_AHRS.h
  
  增加了互补滤波融合算法    EKF_CompleFilter.cpp  EKF_CompleFilter.h
 
  
# 1、下载安装
 1. cd catkin_ws/src
 
 2.  git clone  https://github.com/RuPingCen/pose_estimate.git

 4. catkin_make
