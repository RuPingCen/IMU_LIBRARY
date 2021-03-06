#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "EKF2_AHRS.h"
#include "Convert.h"

using namespace std;
using namespace Eigen;

namespace IMU
{

EKF2_AHRS::EKF2_AHRS():
        mbStopped(false), mbStopRequested(false)
{
	mbFlagInit =0x00;

	cout << "EKF2_AHRS initialization!" << endl;

	q = Eigen::Vector4d(0,0,0,1) ;

	q.normalize();

	P = MatrixXd::Identity(7,7);
	R = 1e-1*MatrixXd::Identity(3,3);
	Q = 1e-2*MatrixXd::Identity(7,7);
	Q.block<4, 4>(0, 0) = 1e-3*MatrixXd::Identity(4,4);
	Q.block<3, 3>(4, 4) = 1e-3*MatrixXd::Identity(3,3);

	P2= 10*MatrixXd::Identity(2,2);
	Q2 = 1e-2*MatrixXd::Identity(2,2);
	R2 = 1e-1*MatrixXd::Identity(2,2);

	state_X = MatrixXd::Zero(7,1);;
	state_X.head<4>() = q;
}

 
void EKF2_AHRS::EKF_Init(const Eigen::Vector3d& gyro,const Eigen::Vector3d& acc,
			  const Eigen::Vector3d& mag)
{
	//旋转方式使用的是RPY，坐标系为以东北天坐标系

	double acc_norm = acc.norm();
	Eigen::Vector3d acc_dat = acc/acc_norm;

	double pitch = -asin(acc_dat(0));
	double roll = atan2(acc_dat(1),acc_dat(2));
	double yaw = 0;

	double mag_norm = mag.norm();
	
	if(mag_norm > 1e-2)
	{
		Eigen::Vector3d mag_dat = mag/mag_norm;

		double m_x = mag_dat(0)*cos(roll)+mag_dat(2)*sin(roll);
		double m_y = mag_dat(0)*sin(pitch)*sin(roll)+ mag_dat(1)*cos(pitch)
						-mag_dat(2)*cos(roll)*sin(pitch); 
		yaw = atan2(m_y,m_x);
	}

	cout<< "Init EKF2  "<< "roll: "<<roll*57.3<<"   pitch"<<pitch*57.3<<"   yaw"<<yaw*57.3<<endl;

	double w = cos(pitch/2)*cos(roll/2)*cos(yaw/2) - sin(pitch/2)*sin(roll/2)*sin(yaw/2);
	double x = cos(roll/2)*cos(yaw/2)*sin(pitch/2) - cos(pitch/2)*sin(roll/2)*sin(yaw/2); 
	double y = cos(pitch/2)*cos(yaw/2)*sin(roll/2) + cos(roll/2)*sin(pitch/2)*sin(yaw/2); 
	double z = cos(pitch/2)*cos(roll/2)*sin(yaw/2) + cos(yaw/2)*sin(pitch/2)*sin(roll/2);

	q = Eigen::Vector4d(x,y,z,w);
	//q = Eigen::Vector4d(0,0,0,1);

	q.normalize();
	//quaternionToEuler(q);
	state_X.head<4>() = q;
}

 
bool EKF2_AHRS::Run(const double& dt,const Eigen::Vector3d& gyro,
			const Eigen::Vector3d& acc,const Eigen::Vector3d& mag)
{
 
	Eigen::Vector3d acc_m = acc;
	acc_m.normalize();

	Eigen::Vector3d mag_m = mag; 
	mag_m.normalize();

	if(mbFlagInit == 0x00)
	{
		EKF_Init(gyro,acc_m,mag_m); // 这部分虽然有问题 但是应该是不影响的
		mbFlagInit = 0x01;

		return true;
	}
 
	Eigen::Matrix4d Omega = Eigen::Matrix4d::Zero();
	Omega.block<3, 3>(0, 0) = -skewSymmetric(gyro);
	Omega.block<3, 1>(0, 3) = gyro;
	Omega.block<1, 3>(3, 0) = -gyro;

	//1. F 
	//Compute discrete transition and noise covariance matrix
	Matrix<double, 7, 7> F = Matrix<double, 7, 7>::Zero();
	F.block<4, 4>(0, 0) = 0.5*Omega;

	Matrix<double, 7, 7> Fdt = F * dt;
	Matrix<double, 7, 7> Fdt_square = Fdt * Fdt;
	Matrix<double, 7, 7> Fdt_cube = Fdt_square * Fdt;
	Matrix<double, 7, 7> Phi = Matrix<double, 7, 7>::Identity() +
		Fdt + 0.5*Fdt_square + (1.0/6.0)*Fdt_cube;

	//Eigen::Matrix<double, 7, 7> Phi = Eigen::Matrix<double, 7, 7>::Identity();
	//Phi.block<4, 4>(0, 0) = Eigen::Matrix<double, 4, 4>::Identity()+Omega*0.5*dt;

	state_X = Phi*state_X;

	Eigen::Vector4d qk= state_X.head<4>();
	qk.normalize();
	state_X.head<4>() = qk;

	P = Phi*P*Phi.transpose()+Q;

	Eigen::Matrix<double, 3, 3> R_bw = quaternionToRotation(q);
 

	Eigen::Matrix<double, 3, 7> H_acc ;
	H_acc <<  2*q(2), -2*q(3), 2*q(0), -2*q(1), 0, 0, 0,
				2*q(3), 2*q(2), 2*q(1), 2*q(0), 0, 0, 0,
			-2*q(0), -2*q(1), 2*q(2), 2*q(3), 0, 0, 0;
	 

	Eigen::Matrix<double, 3, 7> H_thin;
	H_thin.block<3, 7>(0, 0) = H_acc;
 
 
	Eigen::Matrix<double, 3, 1> r_thin;
	r_thin.block<3,1>(0,0)= acc_m - R_bw* Eigen::Vector3d(0, 0, 1);
 

	Eigen::MatrixXd S = H_thin*P*H_thin.transpose() + R;
	Eigen::MatrixXd K = P*H_thin.transpose()*S.inverse();

	// Compute the error of the state.
	state_X = state_X+ K * r_thin;

	Eigen::Vector4d dq_imu = state_X.head<4>();
	dq_imu.normalize();

	state_X.head<4>() = dq_imu;
	q = dq_imu;

	P = (Matrix<double, 7, 7>::Identity()-K*H_thin)*P;
	P= (P +P.transpose()) / 2.0;

	//quaternionToEuler(q);
  
    // ----------------------- 单独更新 yaw -------------------------//
	Eigen::Matrix<double, 2, 2> A2 ;
	A2 << 1,-dt,0,1;
	P2 = A2*A2*A2.transpose() + Q2;
 
	Eigen::Vector3d rpy = quaternionToEuler(dq_imu); 
	rpy = rpy/57.3f;
	double yaw_obs = getYawFromMagnetometer(rpy(0),rpy(1),mag_m);
	Eigen::Vector2d obs2 = Eigen::Vector2d(yaw_obs,0);
 
	Eigen::Matrix<double, 2, 2> H2 =  Matrix<double, 2, 2>::Identity();
	Eigen::Matrix<double, 2, 2> S2 = H2*P2*H2+R2;
	Eigen::Matrix<double, 2, 2> K2 = P2*H2*S2.inverse();
	Eigen::Vector2d e2 = Eigen::Vector2d(rpy(2),0) + K2*(obs2 -Eigen::Vector2d(rpy(2),0));

	P2 = (Matrix<double, 2, 2>::Identity()-K2*H2)*P2;
	P2= (P2 +P2.transpose()) / 2.0;
 
	//cout<<"yaw_obs(2): "<<yaw_obs<<"  yaw_est: "<<rpy(2)<<"  yaw_new:"<<e2(0)*57.3<<endl;

	rpy(2) = e2(0);
	q = eulerToQuaternion(rpy(0),rpy(1),rpy(2));
	q.normalize();
	state_X.head<4>() = q;
  
	return true;
}
double EKF2_AHRS::getYawFromMagnetometer(double roll,double pitch, const Eigen::Vector3d& mag)
{
	double yaw = 0;
	double mag_norm = mag.norm();
	
	if(mag_norm > 1e-2)
	{
		Eigen::Vector3d mag_dat = mag/mag_norm;

		double m_x = mag_dat(0)*cos(roll)+mag_dat(2)*sin(roll);
		double m_y = mag_dat(0)*sin(pitch)*sin(roll)+ mag_dat(1)*cos(pitch)
						-mag_dat(2)*cos(roll)*sin(pitch); 
		yaw = atan2(m_y,m_x);
	}

	return yaw;
}
Eigen::Vector4d EKF2_AHRS::eulerToQuaternion(double roll,double pitch, double yaw)
{
	double w = cos(pitch/2)*cos(roll/2)*cos(yaw/2) - sin(pitch/2)*sin(roll/2)*sin(yaw/2);
	double x = cos(roll/2)*cos(yaw/2)*sin(pitch/2) - cos(pitch/2)*sin(roll/2)*sin(yaw/2); 
	double y = cos(pitch/2)*cos(yaw/2)*sin(roll/2) + cos(roll/2)*sin(pitch/2)*sin(yaw/2); 
	double z = cos(pitch/2)*cos(roll/2)*sin(yaw/2) + cos(yaw/2)*sin(pitch/2)*sin(roll/2);

	return Eigen::Vector4d(x,y,z,w);
}
Eigen::Vector4d EKF2_AHRS::getQuaternion(void)
{
	return q;
}
/**
 *@function 把四元素转换成欧拉角
 *
 */
Eigen::Vector3d EKF2_AHRS::quaternionToEuler(const Eigen::Vector4d& q) 
{
	double q_x = q(0);
	double q_y = q(1);
	double q_z = q(2);
 	double q_w = q(3);

	Eigen::Matrix<double, 3, 3> R_n2b;
	R_n2b <<1 - 2 * (q_z *q_z + q_y * q_y), 2 * (q_x * q_y-q_w * q_z), 2 * (q_x * q_z +q_w * q_y),
			2 * (q_x * q_y +q_w * q_z) ,    1 - 2 * (q_z *q_z + q_x * q_x),    2 * (q_y * q_z-q_w * q_x),
			2 * (q_x * q_z-q_w * q_y),      2 * (q_y * q_z+q_w * q_x),         1 - 2 * (q_x *q_x + q_y * q_y); 

	// 同样都使用RPY的旋转顺序，但是由于在东北天和北东地两个坐标系下
	// X 和 Y轴的方向相反，因此从四元数矩阵到欧拉角的计算方法上也不同
	// 因此存在以下两种计算方法
	double Rad_to_Angle =57.3;
	double roll  = atan2(-R_n2b(2,0),R_n2b(2,2))*Rad_to_Angle;
	double pitch = asin(R_n2b(2,1))*Rad_to_Angle;
	double yaw   = -atan2(R_n2b(0,1),R_n2b(1,1))*Rad_to_Angle; 

	// double roll  = atan2(R_n2b(2,1),R_n2b(2,2))*Rad_to_Angle;
	// double pitch = -asin(R_n2b(2,0))*Rad_to_Angle;
	// double yaw   = atan2(R_n2b(1,0),R_n2b(0,0))*Rad_to_Angle; 
	cout<< "ENU: roll "<<roll<<" pitch:  "<<pitch<<" yaw:  "<<yaw<<endl;

 
	return Eigen::Vector3d(roll,pitch,yaw);
}

void EKF2_AHRS::Param_Change(float noise_R, float noise_Qq,float noise_Qw)
{
	if (isStopped())
	{
		R = noise_R*MatrixXd::Identity(6,6);
		Q = MatrixXd::Identity(7,7);

		Q.block<4, 4>(0, 0) = noise_Qq*MatrixXd::Identity(4,4);
		Q.block<3, 3>(4, 4) = noise_Qw*MatrixXd::Identity(3,3);
	}
}
 
void EKF2_AHRS::RequestStop()
{
	unique_lock<mutex> lock(mMutexStop);
	mbStopRequested = true;
}

void EKF2_AHRS::RequestStart()
{
	unique_lock<mutex> lock(mMutexStop);
	if (mbStopped)
	{
		mbStopped = false;
		mbStopRequested = false;
	}

}

bool EKF2_AHRS::Stop()
{
	unique_lock<mutex> lock(mMutexStop);
	if (mbStopRequested)
	{
		mbStopped = true;
		return true;
	}
	return false;
}

bool EKF2_AHRS::isStopped()
{
	unique_lock<mutex> lock(mMutexStop);
	return mbStopped;
}

void EKF2_AHRS::Release()
{
	unique_lock<mutex> lock(mMutexStop);
	mbStopped = false;
	mbStopRequested = false;
 
	cout << "EKF attitude release " << endl;
}

}