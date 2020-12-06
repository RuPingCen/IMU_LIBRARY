# IMU_LIBRARY
包含IMU设备读取和基于IMU融合算法的姿态估计包

## imu_driver
为IMU设备的驱动包,主要用于读取自制的IMU设备和市面上常见的IMU设备的数据发布到指定的topic上

## pose_estimate
包含了针对MEMS IMU设备的姿态融合算法,如常见的Mahony, Madgwick, EKF 算法
