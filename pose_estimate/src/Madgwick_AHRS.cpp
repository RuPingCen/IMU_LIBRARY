#include <iostream>
#include "Madgwick_AHRS.h"
#include "Convert.h"

using namespace std;
namespace IMU
{
	Madgwick_AHRS::Madgwick_AHRS(double sampleFreq_, double betaDef_):
			mbStopped(false), mbStopRequested(false)
	{
		sampleFreq = sampleFreq_;
		beta = betaDef_; // 2 * proportional gain (Kp)
		q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	 
		q = Eigen::Vector4d(0,0,0,1); // x y z w
		mbInitFilter = false;
		cout<<"Madgwick_AHRS  "<<" sampleFreq: "<<sampleFreq<<"  beta: "<<beta<<endl;
	}
	void Madgwick_AHRS::Madgwick_Init(const Eigen::Vector3d& acc_m,const Eigen::Vector3d& mag_m)
	{
		//旋转方式使用的是RPY，坐标系为以东北天坐标系

		double acc_norm = acc_m.norm();
		Eigen::Vector3d acc_dat = acc_m/acc_norm;

		double pitch = -asin(acc_dat(0));
		double roll = atan2(acc_dat(1),acc_dat(2));
		double yaw = 0;

		double mag_norm = mag_m.norm();
		
		if(mag_norm > 1e-2)
		{
			Eigen::Vector3d mag_dat = mag_m/mag_norm;

			double m_x = mag_dat(0)*cos(roll)+mag_dat(2)*sin(roll);
			double m_y = mag_dat(0)*sin(pitch)*sin(roll)+ mag_dat(1)*cos(pitch)
							-mag_dat(2)*cos(roll)*sin(pitch); 
			yaw = atan2(m_y,m_x);
		}
		//cout<< "acc: "<<acc_m.transpose()<< "mag: "<<mag_m.transpose()<<endl;
		cout<< "Init Madgwick  "<< "roll: "<<roll*57.3<<"   pitch"<<pitch*57.3<<"   yaw"<<yaw*57.3<<endl;

		double w = cos(pitch/2)*cos(roll/2)*cos(yaw/2) - sin(pitch/2)*sin(roll/2)*sin(yaw/2);
		double x = cos(roll/2)*cos(yaw/2)*sin(pitch/2) - cos(pitch/2)*sin(roll/2)*sin(yaw/2); 
		double y = cos(pitch/2)*cos(yaw/2)*sin(roll/2) + cos(roll/2)*sin(pitch/2)*sin(yaw/2); 
		double z = cos(pitch/2)*cos(roll/2)*sin(yaw/2) + cos(yaw/2)*sin(pitch/2)*sin(roll/2);

		q = Eigen::Vector4d(x,y,z,w);
		//q = Eigen::Vector4d(0,0,0,1);
		q.normalize();	 

		q0 = q(3);
		q1 = q(0);
		q2 = q(1);
		q3 = q(2);
	}
 
	Eigen::Vector4d Madgwick_AHRS::Run(const double& dt,const Eigen::Vector3d& gyro_m,
			const Eigen::Vector3d& acc_m)
	{
		sampleFreq = 1.0/dt;
	  
		if(mbInitFilter == false )
		{
			Madgwick_Init(acc_m,Eigen::Vector3d(0,0,0));
			mbInitFilter = true;
			return q;
		}
 
        MadgwickAHRSupdateIMU(gyro_m(0),gyro_m(1),gyro_m(2),acc_m(0),acc_m(1),acc_m(2)); 
		q(0) = q1;
		q(1) = q2;
		q(2) = q3;
		q(3) = q0; 

		//getEulerAngle();
		return q;
	} 
	/**
	 * @function   
	 */ 
	Eigen::Vector4d Madgwick_AHRS::Run(const double& dt,const Eigen::Vector3d& gyro_m,
			const Eigen::Vector3d& acc_m,const Eigen::Vector3d& mag_m)
	{
		sampleFreq = 1.0/dt;
 
		if(mbInitFilter == false)
		{
			Madgwick_Init(acc_m,mag_m);
			mbInitFilter = true;
			return q;
		}
 
		MadgwickAHRSupdate(gyro_m(0),gyro_m(1),gyro_m(2),acc_m(0),acc_m(1),acc_m(2),mag_m(0),mag_m(1),mag_m(2)); 
		q(0) = q1;
		q(1) = q2;
		q(2) = q3;
		q(3) = q0;

		//getEulerAngle();
		return q;
	}

	//====================================================================================================
	// Functions
    // 代码来源 https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
	//---------------------------------------------------------------------------------------------------
	// AHRS algorithm update
   
	void Madgwick_AHRS::MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
		float recipNorm;
		float s0, s1, s2, s3;
		float qDot1, qDot2, qDot3, qDot4;
		float hx, hy;
		float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

		// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
		if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
			MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
			return;
		}

		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalise accelerometer measurement
			recipNorm = 1.0/sqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;   

			// Normalise magnetometer measurement
			recipNorm = 1.0/sqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0f * q0 * mx;
			_2q0my = 2.0f * q0 * my;
			_2q0mz = 2.0f * q0 * mz;
			_2q1mx = 2.0f * q1 * mx;
			_2q0 = 2.0f * q0;
			_2q1 = 2.0f * q1;
			_2q2 = 2.0f * q2;
			_2q3 = 2.0f * q3;
			_2q0q2 = 2.0f * q0 * q2;
			_2q2q3 = 2.0f * q2 * q3;
			q0q0 = q0 * q0;
			q0q1 = q0 * q1;
			q0q2 = q0 * q2;
			q0q3 = q0 * q3;
			q1q1 = q1 * q1;
			q1q2 = q1 * q2;
			q1q3 = q1 * q3;
			q2q2 = q2 * q2;
			q2q3 = q2 * q3;
			q3q3 = q3 * q3;

			// Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
			_2bx = sqrt(hx * hx + hy * hy);
			_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
			_4bx = 2.0f * _2bx;
			_4bz = 2.0f * _2bz;

			// Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			recipNorm = 1.0/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
		q0 += qDot1 * (1.0f / sampleFreq);
		q1 += qDot2 * (1.0f / sampleFreq);
		q2 += qDot3 * (1.0f / sampleFreq);
		q3 += qDot4 * (1.0f / sampleFreq);

		// Normalise quaternion
		recipNorm = 1.0/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
	}
 
	void Madgwick_AHRS::MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
		float recipNorm;
		float s0, s1, s2, s3;
		float qDot1, qDot2, qDot3, qDot4;
		float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalise accelerometer measurement
			recipNorm = 1.0/sqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;   

			// Auxiliary variables to avoid repeated arithmetic
			_2q0 = 2.0f * q0;
			_2q1 = 2.0f * q1;
			_2q2 = 2.0f * q2;
			_2q3 = 2.0f * q3;
			_4q0 = 4.0f * q0;
			_4q1 = 4.0f * q1;
			_4q2 = 4.0f * q2;
			_8q1 = 8.0f * q1;
			_8q2 = 8.0f * q2;
			q0q0 = q0 * q0;
			q1q1 = q1 * q1;
			q2q2 = q2 * q2;
			q3q3 = q3 * q3;

			// Gradient decent algorithm corrective step
			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
			s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
			s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
			recipNorm = 1.0/sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
		q0 += qDot1 * (1.0f / sampleFreq);
		q1 += qDot2 * (1.0f / sampleFreq);
		q2 += qDot3 * (1.0f / sampleFreq);
		q3 += qDot4 * (1.0f / sampleFreq);

		// Normalise quaternion
		recipNorm = 1.0/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;
	}

	 
	//====================================================================================================

 	Eigen::Vector4d Madgwick_AHRS::getQuaternion(void)
	{
		return q;
	}
	Eigen::Vector3d Madgwick_AHRS::quaternionToEuler(const Eigen::Vector4d& q) 
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
		// double roll  = atan2(-R_n2b(2,0),R_n2b(2,2))*Rad_to_Angle;
		// double pitch = asin(R_n2b(2,1))*Rad_to_Angle;
		// double yaw   = -atan2(R_n2b(0,1),R_n2b(1,1))*Rad_to_Angle; 

		double roll  = atan2(R_n2b(2,1),R_n2b(2,2))*Rad_to_Angle;
		double pitch = -asin(R_n2b(2,0))*Rad_to_Angle;
		double yaw   = atan2(R_n2b(1,0),R_n2b(0,0))*Rad_to_Angle; 
		cout<< "ENU: roll "<<roll<<" pitch:  "<<pitch<<" yaw:  "<<yaw<<endl;

		return Eigen::Vector3d(roll,pitch,yaw);
	}
	Eigen::Vector3d Madgwick_AHRS::getEulerAngle(void)
	{
		 
		return quaternionToEuler(q);
	}
	void Madgwick_AHRS::paramsChange(double sampleFreq_, double betaDef_)
	{
		if (isStopped())
		{
			sampleFreq = sampleFreq_;
			beta = betaDef_;
			q = Eigen::Vector4d(0,0,0,1);
		}
	}
 	
	void Madgwick_AHRS::RequestStop()
	{
		unique_lock<mutex> lock(mMutexStop);
		mbStopRequested = true;
	}

	void Madgwick_AHRS::RequestStart()
	{
		unique_lock<mutex> lock(mMutexStop);
		if (mbStopped)
		{
			mbStopped = false;
			mbStopRequested = false;
		}

	}

	bool Madgwick_AHRS::Stop()
	{
		unique_lock<mutex> lock(mMutexStop);
		if (mbStopRequested)
		{
			mbStopped = true;
			return true;
		}
		return false;
	}

	bool Madgwick_AHRS::isStopped()
	{
		unique_lock<mutex> lock(mMutexStop);
		return mbStopped;
	}

	void Madgwick_AHRS::Release()
	{
		unique_lock<mutex> lock(mMutexStop);
		mbStopped = false;
		mbStopRequested = false;
 
		cout << "Madgwick attitude release " << endl;
	}


 
}
