#ifndef MADGWICK_AHRS_H_
#define MADGWICK_AHRS_H_

#include <iostream>
#include <vector>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "TypeDefs.h"

 

using namespace std;
namespace IMU
{
 

class Madgwick_AHRS
{
public:
	Madgwick_AHRS(double sampleFreq_, double betaDef_);
	void Madgwick_Init(const Eigen::Vector3d& acc_m,const Eigen::Vector3d& mag_m);
	Eigen::Vector4d Run(const double& dt,const Eigen::Vector3d& gyro_m,
			const Eigen::Vector3d& acc_m);

	Eigen::Vector4d Run(const double& dt,const Eigen::Vector3d& gyro_m,
			const Eigen::Vector3d& acc_m,const Eigen::Vector3d& mag_m);

	void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
	 
	void paramsChange(double sampleFreq_, double betaDef_);
	Eigen::Vector3d quaternionToEuler(const Eigen::Vector4d& q); 
	Eigen::Vector3d getEulerAngle(void);
	Eigen::Vector4d getQuaternion(void);

	void Release();
	void RequestStop();
	void RequestStart();
	bool Stop();
	bool isStopped();
	 
private:
 
	bool mbInitFilter;
 
    float sampleFreq,beta; // beta = 2 * proportional gain (Kp)
	float q0,q1,q2,q3;	// quaternion [qw qx qy qz]
	Eigen::Vector4d q; // [qx qy qz qw]

	bool mbStopped;
	bool mbStopRequested;
	std::mutex mMutexStop;
};

} //namesapce IMU


#endif