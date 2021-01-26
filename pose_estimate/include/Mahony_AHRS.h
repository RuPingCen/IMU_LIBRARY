#ifndef MAHONY_AHRS_H_
#define MAHONY_AHRS_H_

#include <iostream>
#include <vector>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "TypeDefs.h"

#include "Iir.h"

#ifdef _WIN32
#include "windows.h"
#else 
#include <unistd.h>  
#endif

using namespace std;
namespace IMU
{

class Mahony_AHRS
{
public:
	Mahony_AHRS(Vector_2 PI, double dt);
	void Mahony_Init(const Eigen::Vector3d& acc_m,const Eigen::Vector3d& mag_m);
	Eigen::Vector4d Run(const double& dt,const Eigen::Vector3d& gyro_m,
			const Eigen::Vector3d& acc_m);

	Eigen::Vector4d Run(const double& dt,const Eigen::Vector3d& gyro_m,
			const Eigen::Vector3d& acc_m,const Eigen::Vector3d& mag_m);
	Eigen::Vector4d Run2(const double& dt,const Eigen::Vector3d& gyro_m,
			const Eigen::Vector3d& acc_m,const Eigen::Vector3d& mag_m);

	void Mahony_Estimate(void);
	void Mahony_Estimate2(void);
	void Mahony_Estimate3(void);

	void NewValues(const Eigen::Vector3d& gyro_m, const Eigen::Vector3d& acc_m, const Eigen::Vector3d& mag_m);
	void paramsChange(Vector_2 PI, double dt);
	  
	Eigen::Vector3d getEulerAngle(void);
	Eigen::Vector4d getQuaternion(void);

	void Release();
	void RequestStop();
	void RequestStart();
	bool Stop();
	bool isStopped();
	 
private:
    Eigen::Vector3d gyro, acc, mag;
	bool mbInitFilter;
	int miCounter;

	double Kp, Ki,deltaT; //四元数互补滤波增益系数，积分系数 和积分时间
	Eigen::Vector3d Integ_angular;
	Eigen::Vector4d q; // [qx qy qz qw]

	bool mbStopped;
	bool mbStopRequested;
	std::mutex mMutexStop;

	//const int order = 4;
	Iir::Butterworth::LowPass<4> F_LowPass;
	Iir::Butterworth::HighPass<4> F_HighPass;
};

} //namesapce IMU


#endif