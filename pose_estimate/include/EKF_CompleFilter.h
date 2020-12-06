#ifndef EKF_COMPLEFILTER_H_
#define EKF_COMPLEFILTER_H_

#include <iostream>
#include <vector>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Geometry> 
#include "TypeDefs.h"


using namespace std;
namespace IMU
{

class EKF_CompleFilter
{

public:
    EKF_CompleFilter();
    
  
    bool Run(const double& dt,const Eigen::Vector3d& gyro,
                const Eigen::Vector3d& acc,const Eigen::Vector3d& mag);
 
    void Release();
    void RequestStop();
    void RequestStart();
    bool Stop();
    bool isStopped();

private:
 
    Eigen::Vector2d xx);
    Eigen::Matrix<double, 2, 2> PP , QQ, RR;

    bool mbFlagInit;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
};

} //namespace IMU

#endif