#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "EKF_9Axis_AHRS.h"
#include "Convert.h"

using namespace std;
using namespace Eigen;
 
namespace IMU
{

EKF_CompleFilter::EKF_CompleFilter():
        mbStopped(false), mbStopRequested(false)
{
	mbFlagInit =0x00;

	cout << "System initialization!" << endl;
 
	xx = Eigen::Vector2d(0,0);
	PP = 5*MatrixXd::Identity(2,2);
	QQ = 1e-1*MatrixXd::Identity(2,2);
	RR = MatrixXd::Identity(2,2);
}

 
 
/**
 *@function  
 * 这个EKF是将单片机上的EKF 移植过来以后的   输入是加速度和陀螺仪（单位是°/s）
 * 先使用加速度计算pitch角度和roll角度的观察值 单位是度， 利用陀螺仪积分的值作为预测值 单位是度
 * 利用EKF做观测器进行融合，输出滤波以后的角度
 * @ crp
 * @ 2020-10-15
 */
bool EKF_CompleFilter::Run(const double& dt,const Eigen::Vector3d& gyro,const Eigen::Vector3d& acc) 
{
	Eigen::Vector3d z = acc/acc.norm();
	double acc_angle_x = asin(z(0))*57.3; //pitch
	double acc_angle_y = atan2(-z(1),z(2))*57.3; //roll
	//cout<<"EKF2: "<<"acc_angle_x: "<<acc_angle_x<<"  acc_angle_y: "<<acc_angle_y<<endl;
	//cout<<"EKF2: "<<"gx: "<<gyro(0)<<"  gy: "<<gyro(1)<<endl;

	Matrix<double, 2, 2> Phi=Matrix<double, 2, 2>::Identity();
	// Phi<< 1+gyro(1)*dt,0,0,1+gyro(0)*dt;
	Matrix<double, 2, 2> G;
	G<< dt,0,0,dt;

	xx = xx+G*Eigen::Vector2d(gyro(1),gyro(0)); 
	PP = Phi*PP*Phi.transpose()+QQ;

	Eigen::Vector2d r_thin = Eigen::Vector2d(acc_angle_x - xx(0),acc_angle_y - xx(1));
	//cout<< "r_thin: " << r_thin.transpose()<<endl;

	Eigen::Matrix<double, 2, 2> H_thin = Matrix<double, 2, 2>::Identity();

	Eigen::MatrixXd S = H_thin*PP*H_thin.transpose() + RR;
	Eigen::MatrixXd K = PP*H_thin.transpose()*S.inverse();

	xx = xx+K * r_thin;
	//   //cout<< "delta_x: " << delta_x.transpose()<<endl;

	PP = (Matrix<double, 2, 2>::Identity()-K*H_thin)*PP;
	PP= (PP +PP.transpose()) / 2.0;

	cout<<"EKF2: "<<"x: "<<xx(0)<<"y: "<<xx(1)<<endl;
 
	return true;
}

 
 
void EKF_CompleFilter::RequestStop()
{
	unique_lock<mutex> lock(mMutexStop);
	mbStopRequested = true;
}

void EKF_CompleFilter::RequestStart()
{
	unique_lock<mutex> lock(mMutexStop);
	if (mbStopped)
	{
		mbStopped = false;
		mbStopRequested = false;
	}

}

bool EKF_CompleFilter::Stop()
{
	unique_lock<mutex> lock(mMutexStop);
	if (mbStopRequested)
	{
		mbStopped = true;
		return true;
	}
	return false;
}

bool EKF_CompleFilter::isStopped()
{
	unique_lock<mutex> lock(mMutexStop);
	return mbStopped;
}

void EKF_CompleFilter::Release()
{
	unique_lock<mutex> lock(mMutexStop);
	mbStopped = false;
	mbStopRequested = false;
 
	cout << "EKF  CompleFilter release " << endl;
}

}