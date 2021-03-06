#ifndef EKF2_AHRS_H_
#define EKF2_AHRS_H_

#include <iostream>
#include <vector>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Geometry> 
 


using namespace std;
namespace IMU
{

class EKF2_AHRS
{

public:
    EKF2_AHRS();
    
    void EKF_Init(const Eigen::Vector3d& gyro,const Eigen::Vector3d& acc,
                const Eigen::Vector3d& mag);

    bool Run(const double& dt,const Eigen::Vector3d& gyro,
                const Eigen::Vector3d& acc,const Eigen::Vector3d& mag);
  
    double getYawFromMagnetometer(double roll,double pitch, const Eigen::Vector3d& mag);
    Eigen::Vector4d eulerToQuaternion(double roll,double pitch, double yaw);

    Eigen::Vector4d getQuaternion(void);

    Eigen::Vector3d quaternionToEuler(const Eigen::Vector4d& q);
    void Param_Change(float noise_R, float noise_Qq,float noise_Qw);

    void Release();
    void RequestStop();
    void RequestStart();
    bool Stop();
    bool isStopped();

private:
 
    Eigen::Vector4d q;
    Eigen::Matrix<double, 7, 1> state_X;
    Eigen::Matrix<double, 7, 7> P,Q ;
    Eigen::Matrix<double, 3, 3> R; // 当没有磁力计时候，此时观测矩阵的维度为3

    Eigen::Matrix<double, 2, 2> P2,Q2,R2;
    bool mbFlagInit;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
};

} //namespace IMU

#endif