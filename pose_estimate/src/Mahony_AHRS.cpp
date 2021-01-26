#include <iostream>
#include <ros/ros.h>

#include "Mahony_AHRS.h"
#include "Convert.h"

#include "Iir.h"

using namespace std;



#define  IMU_FIFO_Length 10
double IMU_FIFO[9][IMU_FIFO_Length+1];

namespace IMU
{
	Mahony_AHRS::Mahony_AHRS(Vector_2 PI, double dt):
			mbStopped(false), mbStopRequested(false)
	{
		Kp = PI(0);
		Ki = PI(1);
		deltaT = dt;
		Integ_angular = Vector_3::Zero();
		q = Eigen::Vector4d(0,0,0,1); // x y z w
		mbInitFilter = false;
		miCounter =0;

		
		uint16_t samplingrate =100;
		uint16_t cutoff_frequence =50;
		F_LowPass.setup(samplingrate,cutoff_frequence);
		cutoff_frequence =2;
		F_HighPass.setup(samplingrate,cutoff_frequence);

		cout<<"Mahony_AHRS  Kp: "<<Kp<<"  Ki:"<<Ki<<endl;
	
	}
	void Mahony_AHRS::Mahony_Init(const Eigen::Vector3d& acc_m,const Eigen::Vector3d& mag_m)
	{
		//旋转方式使用的是RPY，坐标系为以东北天坐标系

		double acc_norm = acc_m.norm();

		if(acc_norm < 0.8)
		{
			cout<<"acc measurements is error, exit ..."<<endl;
			return ;
		}
		Eigen::Vector3d acc_dat = acc/acc_norm;

		double pitch = -asin(acc_dat(0));
		double roll = atan2(acc_dat(1),acc_dat(2));
		double yaw = 0;

		double mag_norm = mag_m.norm();
		
		if(mag_norm > 1e-2)
		{
			Eigen::Vector3d mag_dat = mag/mag_norm;

			double m_x = mag_dat(0)*cos(roll)+mag_dat(2)*sin(roll);
			double m_y = mag_dat(0)*sin(pitch)*sin(roll)+ mag_dat(1)*cos(pitch)
							-mag_dat(2)*cos(roll)*sin(pitch); 
			yaw = atan2(m_y,m_x);
		}
		//cout<< "acc: "<<acc_m.transpose()<< "mag: "<<mag_m.transpose()<<endl;
		cout<< "Init Mahony  "<< "roll: "<<roll*57.3<<"   pitch"<<pitch*57.3<<"   yaw"<<yaw*57.3<<endl;

		double w = cos(pitch/2)*cos(roll/2)*cos(yaw/2) - sin(pitch/2)*sin(roll/2)*sin(yaw/2);
		double x = cos(roll/2)*cos(yaw/2)*sin(pitch/2) - cos(pitch/2)*sin(roll/2)*sin(yaw/2); 
		double y = cos(pitch/2)*cos(yaw/2)*sin(roll/2) + cos(roll/2)*sin(pitch/2)*sin(yaw/2); 
		double z = cos(pitch/2)*cos(roll/2)*sin(yaw/2) + cos(yaw/2)*sin(pitch/2)*sin(roll/2);

		q = Eigen::Vector4d(x,y,z,w);
		//q = Eigen::Vector4d(0,0,0,1);
		q.normalize();	 
	}
	/**
	 * 输入参数dt   积分时间
	 * 输入参数gx，gy，gz    三个轴的角速度(弧度/秒)
	 * 输入参数ax，ay，az    三个轴的加速度数据
	 * 输出参数mx，my，mz    三轴的电子罗盘数据
	 */ 
	Eigen::Vector4d Mahony_AHRS::Run(const double& dt,const Eigen::Vector3d& gyro_m,
			const Eigen::Vector3d& acc_m)
	{
		deltaT = dt;

		//gyro = gyro_m;
		//acc = acc_m;
		mag = Eigen::Vector3d(0,0,0);

		NewValues(gyro_m, acc_m, mag);
		gyro = Eigen::Vector3d(IMU_FIFO[3][IMU_FIFO_Length],IMU_FIFO[4][IMU_FIFO_Length],IMU_FIFO[5][IMU_FIFO_Length]);
		acc = Eigen::Vector3d(IMU_FIFO[0][IMU_FIFO_Length],IMU_FIFO[1][IMU_FIFO_Length],IMU_FIFO[2][IMU_FIFO_Length]);
		

		if(mbInitFilter == false )
		{
			Mahony_Init(acc_m,Eigen::Vector3d(0,0,0));
			mbInitFilter = true;
			return q;
		}
		Mahony_Estimate2();
        //getEulerAngle();
		return q;
	} 
	/**
	 * @function 当前 Mahony_Estimate 在不加入磁力计的时候还是能准确估计到位姿
	 * 			 但是加入了磁力计以后就出现了问题，平面上转动Z轴的时候，roll角
	 * 			 会变化。
	 */ 
	Eigen::Vector4d Mahony_AHRS::Run(const double& dt,const Eigen::Vector3d& gyro_m,
			const Eigen::Vector3d& acc_m,const Eigen::Vector3d& mag_m)
	{
		deltaT = dt;
		 gyro = gyro_m;
		// acc = acc_m;
		// mag = mag_m;
		NewValues(gyro_m, acc_m, mag_m);
		//gyro = Eigen::Vector3d(IMU_FIFO[3][IMU_FIFO_Length],IMU_FIFO[4][IMU_FIFO_Length],IMU_FIFO[5][IMU_FIFO_Length]);
		acc = Eigen::Vector3d(IMU_FIFO[0][IMU_FIFO_Length],IMU_FIFO[1][IMU_FIFO_Length],IMU_FIFO[2][IMU_FIFO_Length]);
		mag = Eigen::Vector3d(IMU_FIFO[6][IMU_FIFO_Length],IMU_FIFO[7][IMU_FIFO_Length],IMU_FIFO[8][IMU_FIFO_Length]);

		//mag = Eigen::Vector3d(0,0,0);
		if(mbInitFilter == false || miCounter<15)
		{
			
			if(miCounter<15)
			{
				NewValues(gyro_m, acc_m, mag_m);
			}
			else
			{
				Mahony_Init(acc_m,mag_m);
				mbInitFilter = true;
			}
			miCounter++;			
			return q;
		}

		//Mahony_Estimate();
		//getEulerAngle();
		Mahony_Estimate();
		
		return q;
	}

Eigen::Vector4d Mahony_AHRS::Run2(const double& dt,const Eigen::Vector3d& gyro_m,
			const Eigen::Vector3d& acc_m,const Eigen::Vector3d& mag_m)
	{
		deltaT = dt;
		 gyro = gyro_m;
		// acc = acc_m;
		// mag = mag_m;
		NewValues(gyro_m, acc_m, mag_m);
		//gyro = Eigen::Vector3d(IMU_FIFO[3][IMU_FIFO_Length],IMU_FIFO[4][IMU_FIFO_Length],IMU_FIFO[5][IMU_FIFO_Length]);
		acc = Eigen::Vector3d(IMU_FIFO[0][IMU_FIFO_Length],IMU_FIFO[1][IMU_FIFO_Length],IMU_FIFO[2][IMU_FIFO_Length]);
		mag = Eigen::Vector3d(IMU_FIFO[6][IMU_FIFO_Length],IMU_FIFO[7][IMU_FIFO_Length],IMU_FIFO[8][IMU_FIFO_Length]);

		//mag = Eigen::Vector3d(0,0,0);
		if(mbInitFilter == false || miCounter<15)
		{
			
			if(miCounter<15)
			{
				NewValues(gyro_m, acc_m, mag_m);
			}
			else
			{
				Mahony_Init(acc_m,mag_m);
				mbInitFilter = true;
			}
			miCounter++;			
			return q;
		}

		//Mahony_Estimate();
		//getEulerAngle();
		Mahony_Estimate2();
		
		return q;
	}

	void Mahony_AHRS::Mahony_Estimate(void)
	{
 
 		acc.normalize(); 
		 
		q.normalize();
		Eigen::Matrix<double, 3, 3> R_bw = quaternionToRotation(q);
		
		Vector_3 error_acc, error_mag, mag_ned, error_sum;
		error_acc = R_bw*Eigen::Vector3d(0,0,1);
		error_acc = acc.cross(error_acc);//向量叉乘
		//error_acc = skewSymmetric(acc)*error_acc;
		error_sum = error_acc;

		if(mag.norm() > 1e-6)
		{
			mag.normalize(); 
			mag_ned = R_bw.transpose()*mag;
			error_mag << sqrt(mag_ned(0)*mag_ned(0) + mag_ned(1)*mag_ned(1)), 0, mag_ned(2);
			error_mag = R_bw*error_mag;
			error_mag = mag.cross(error_mag);
			//error_mag = skewSymmetric(mag)*error_mag;
			error_sum += error_mag;
		}
   
		if(error_sum(0) != 0.0f && error_sum(1) != 0.0f && error_sum(2) != 0.0f)
		{	
			 
				// get the modify value from the error
				Integ_angular = Integ_angular + 0.5*Ki*deltaT*error_sum;
				gyro = gyro + Kp*error_sum + Integ_angular;
			 
			 
		}
 
		Eigen::Matrix4d Omega = Eigen::Matrix4d::Zero();
		Omega.block<3, 3>(0, 0) = -skewSymmetric(gyro);
		Omega.block<3, 1>(0, 3) = gyro;
		Omega.block<1, 3>(3, 0) = -gyro;

		q = (Eigen::Matrix<double, 4, 4>::Identity() +0.5*deltaT*Omega)*q;
		q.normalize();
			 
	}
	/**
	 *@ function 利用加速度的观察信息 动态调整Kp的值 
	 * 			 实现变权重的姿态融合算法
	 * 
	 */
	void Mahony_AHRS::Mahony_Estimate2(void)
	{
		double acc_norm = acc.norm();
 		acc.normalize(); 
		 
		q.normalize();
		Eigen::Matrix<double, 3, 3> R_bw = quaternionToRotation(q);
		
		Vector_3 error_acc, error_mag, mag_ned, error_sum;
		error_acc = R_bw*Eigen::Vector3d(0,0,1);
		error_acc = acc.cross(error_acc);//向量叉乘
		//error_acc = skewSymmetric(acc)*error_acc;
		error_sum = error_acc;

		if(mag.norm() > 1e-6)
		{
			mag.normalize(); 
			mag_ned = R_bw.transpose()*mag;
			error_mag << sqrt(mag_ned(0)*mag_ned(0) + mag_ned(1)*mag_ned(1)), 0, mag_ned(2);
			error_mag = R_bw*error_mag;
			error_mag = mag.cross(error_mag);
			//error_mag = skewSymmetric(mag)*error_mag;
			error_sum += error_mag;
		}
 
		acc_norm = F_LowPass.filter(acc_norm);
		if(acc_norm<0) acc_norm= -acc_norm;

		acc_norm = F_HighPass.filter(acc_norm);
		if(acc_norm<0) acc_norm= -acc_norm;
 
		uint8_t stationary = 1;
		
		if(acc_norm<1.0015*9.8) //	表示平稳状态
		{
			stationary = 1;
		}
		else
		{
			stationary= 0;
		}

		if(error_sum(0) != 0.0f && error_sum(1) != 0.0f && error_sum(2) != 0.0f)
		{	
			if(stationary == 1)
			{
				// get the modify value from the error
				Integ_angular = Integ_angular + 0.5*Ki*deltaT*error_sum;
				gyro = gyro + Kp*error_sum + Integ_angular;
			}
			else
			{
				cout<<"acc_norm: "<<acc_norm<<endl;
				
				// get the modify value from the error
				Integ_angular = Integ_angular + 0.5*Ki*deltaT*error_sum;

				//gyro = gyro + Integ_angular;		

				float tem_Kp = Kp/(exp(acc_norm*acc_norm));
				gyro = gyro +  tem_Kp*error_sum+Integ_angular;		 
			}
			 
		}
 
		Eigen::Matrix4d Omega = Eigen::Matrix4d::Zero();
		Omega.block<3, 3>(0, 0) = -skewSymmetric(gyro);
		Omega.block<3, 1>(0, 3) = gyro;
		Omega.block<1, 3>(3, 0) = -gyro;

		q = (Eigen::Matrix<double, 4, 4>::Identity() +0.5*deltaT*Omega)*q;
		q.normalize();
			 
	}
	/**
	 * @function 这部分的代码是直接移植的单片机上的代码
	 * 坐标系为 北东地
	 * 
	 */ 
	void Mahony_AHRS::Mahony_Estimate3(void)
	{
 
		float gx = gyro(0);
		float gy = gyro(1);
		float gz = gyro(2);
		float ax = acc(0);
		float ay = acc(1);
		float az = acc(2);
		float mx = mag(0);
		float my = mag(1);
		float mz = mag(2);

		static float exInt=0.0f, eyInt=0.0f, ezInt=0.0f;  // 误差积分
		float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f; // 全局四元数
		q0 = q(3);
		q1 = q(0);
		q2 = q(1);
		q3 = q(2);
		//float Kp  =  10.0f;    
		//float Ki   =    0.01f;    
		float halfT =  deltaT/2;   

		float norm;
		float hx, hy, hz, bx, bz;
		float vx, vy, vz, wx, wy, wz;
		float ex, ey, ez;
		float tempq0,tempq1,tempq2,tempq3;

		float q0q0 = q0*q0;// 先把这些用得到的值算好
		float q0q1 = q0*q1;
		float q0q2 = q0*q2;
		float q0q3 = q0*q3;
		float q1q1 = q1*q1;
		float q1q2 = q1*q2;
		float q1q3 = q1*q3;
		float q2q2 = q2*q2;   
		float q2q3 = q2*q3;
		float q3q3 = q3*q3;          

		norm = sqrt(ax*ax + ay*ay + az*az);       
		ax = ax /double(norm);
		ay = ay /double(norm);
		az = az /double(norm);

		vx = 2*(q1q3 - q0q2);
		vy = 2*(q0q1 + q2q3);
		vz = q0q0 - q1q1 - q2q2 + q3q3;
 

		if(mx == 0 && my == 0 && mz == 0)
		{
			ex = (ay*vz - az*vy);
			ey = (az*vx - ax*vz);
			ez = (ax*vy - ay*vx);
		}
		else
		{
			norm = sqrt(mx*mx + my*my + mz*mz);          
			mx = mx/double(norm);
			my = my/double(norm);
			mz = mz/double(norm);

			hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2); 
			hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
			hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);
				
			bx = sqrt((hx*hx) + (hy*hy));
			bz = hz;     

			wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
			wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
			wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  

			ex = (ay*vz - az*vy) + (my*wz - mz*wy);
			ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
			ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
		}
		if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
		{
				exInt = exInt + ex * Ki * halfT;
				eyInt = eyInt + ey * Ki * halfT;
				ezInt = ezInt + ez * Ki * halfT;
				
				gx = gx + Kp*ex + exInt;// 用叉积误差来做PI修正陀螺零偏
				gy = gy + Kp*ey + eyInt;
				gz = gz + Kp*ez + ezInt;
		}
  
		tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
		tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
		tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
		tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

		// 四元数规范化
		norm = sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
		q0 = tempq0 /double(norm);
		q1 = tempq1 /double(norm);
		q2 = tempq2 /double(norm);
		q3 = tempq3 /double(norm);

		q(0) = q1;
		q(1) = q2;
		q(2) = q3;
		q(3) = q0;

		//double Rad_to_Angle =57.3;
		 
		// double yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* Rad_to_Angle; // yaw
		// double pitch = -asin(-2 * q1 * q3 + 2 * q0 * q2)* Rad_to_Angle; // pitch
		// double roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1)* Rad_to_Angle; // roll

		// cout<< "Mahony2: roll "<<roll<<" pitch:  "<<pitch<<" yaw:  "<<yaw<<endl;
	 
	}

	Eigen::Vector4d Mahony_AHRS::getQuaternion(void)
	{
		return q;
	}
 
	Eigen::Vector3d Mahony_AHRS::getEulerAngle(void)
	{
		 
		return quaternionToEuler(q);
	}
	
void Mahony_AHRS::NewValues(const Eigen::Vector3d& gyro_m, const Eigen::Vector3d& acc_m, const Eigen::Vector3d& mag_m)
{
	unsigned char i ;
	double sum=0;
	for(i=1;i<IMU_FIFO_Length;i++)
	{	//FIFO 操作
		IMU_FIFO[0][i-1]=IMU_FIFO[0][i];
		IMU_FIFO[1][i-1]=IMU_FIFO[1][i];
		IMU_FIFO[2][i-1]=IMU_FIFO[2][i];
		IMU_FIFO[3][i-1]=IMU_FIFO[3][i];
		IMU_FIFO[4][i-1]=IMU_FIFO[4][i];
		IMU_FIFO[5][i-1]=IMU_FIFO[5][i];
		IMU_FIFO[6][i-1]=IMU_FIFO[6][i];
		IMU_FIFO[7][i-1]=IMU_FIFO[7][i];
		IMU_FIFO[8][i-1]=IMU_FIFO[8][i];
	}
	IMU_FIFO[0][IMU_FIFO_Length-1]=acc_m(0);//将新的数据放置到 数据的最后面
	IMU_FIFO[1][IMU_FIFO_Length-1]=acc_m(1);
	IMU_FIFO[2][IMU_FIFO_Length-1]=acc_m(2);
	IMU_FIFO[3][IMU_FIFO_Length-1]=gyro_m(0);
	IMU_FIFO[4][IMU_FIFO_Length-1]=gyro_m(1);
	IMU_FIFO[5][IMU_FIFO_Length-1]=gyro_m(2);
	IMU_FIFO[6][IMU_FIFO_Length-1]=mag_m(0);
	IMU_FIFO[7][IMU_FIFO_Length-1]=mag_m(1);
	IMU_FIFO[8][IMU_FIFO_Length-1]=mag_m(2);

	for(uint8_t j=0;j<9;j++)
	{
		sum=0;
		for(i=0;i<IMU_FIFO_Length;i++)
		{	
			sum+=IMU_FIFO[j][i];//求当前数组的合，再取平均值
		}
		
		IMU_FIFO[j][IMU_FIFO_Length]=sum/((double)IMU_FIFO_Length);
	}
}
	
	void Mahony_AHRS::paramsChange(Vector_2 PI, double dt)
	{
		if (isStopped())
		{
			Kp = PI(0);
			Ki = PI(1);
			deltaT = dt;
			Integ_angular = Vector_3::Zero();
			q = Eigen::Vector4d(0,0,0,1);
		}
	}
	void Mahony_AHRS::RequestStop()
	{
		unique_lock<mutex> lock(mMutexStop);
		mbStopRequested = true;


	}

	void Mahony_AHRS::RequestStart()
	{
		unique_lock<mutex> lock(mMutexStop);
		if (mbStopped)
		{
			mbStopped = false;
			mbStopRequested = false;
		}

	}

	bool Mahony_AHRS::Stop()
	{
		unique_lock<mutex> lock(mMutexStop);
		if (mbStopRequested)
		{
			mbStopped = true;
			return true;
		}
		return false;
	}

	bool Mahony_AHRS::isStopped()
	{
		unique_lock<mutex> lock(mMutexStop);
		return mbStopped;
	}

	void Mahony_AHRS::Release()
	{
		unique_lock<mutex> lock(mMutexStop);
		mbStopped = false;
		mbStopRequested = false;
		
		Integ_angular = Vector_3::Zero();
		cout << "Mahony attitude release " << endl;
	}


 
}