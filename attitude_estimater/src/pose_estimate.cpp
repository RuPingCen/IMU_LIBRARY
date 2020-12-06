/**
 * @function 读取 /imu /mag /gps的话题的数据进行姿态融合
 * 
 * maker:crp
 * 2020-11-6
 */

#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Geometry> 
#include <chrono>
#include <locale>
#include <tuple>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <boost/assert.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
 

extern "C" {
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h> //  close
#include <string.h> //  strerror
}

#include <serial/serial.h>
#include <std_msgs/String.h>


#include <cmath>
#include <Eigen/Dense>


#include "Convert.h"
#include "EKF_AHRS.h"
#include "Mahony_AHRS.h"
#include "Madgwick_AHRS.h"

 

using namespace std;
using namespace Eigen;
using namespace IMU;

std::mutex imuMutex,magMutex,gpsMutex;
std::mutex ekfMutex;

uint32_t seq_counter=0;

ros::Subscriber sub_imu,sub_mag,sub_gps;
ros::Publisher debug_pub;


typedef struct
{
	float 	ax,ay,az;
	float 	gx,gy,gz;
	float 	mx,my,mz;

	float roll,pitch,yaw;
	float temp;
	float qw,qx,qy,qz;
	
	uint8_t IMUFlag;
	uint8_t GPSFlag;	

	double press,high;
	double GPSLon,GPSLat;
	double GPSHeight,GPSYaw,GPSV;
	double GPSSN,GPSPDOP,GPSHDOP,GPSVDOP;
}imu_measure_t;

imu_measure_t imu;
 
void sub_imu_callback(const sensor_msgs::Imu::ConstPtr imu_msg);
void sub_mag_callback(const sensor_msgs::MagneticField::ConstPtr mag_msg);
void sub_gps_callback(const sensor_msgs::NavSatFix::ConstPtr gps_msg);
 

int main(int argc,char** argv)
{
	chrono::steady_clock::time_point t_now,t_last; 
	string sub_imu_topic,sub_mag_topic,sub_gps_topic;
	bool use_mag = false;

	ros::init(argc, argv, "pose_estimate");
	ros::NodeHandle n("~");


	n.param<std::string>("sub_imu_topic", sub_imu_topic, "/imu");
	n.param<std::string>("sub_mag_topic", sub_mag_topic, "/mag");
	n.param<std::string>("sub_gps_topic", sub_gps_topic, "/gps");
	n.param<bool>("use_mag", use_mag, false);

	sub_imu = n.subscribe(sub_imu_topic, 10, sub_imu_callback);
	sub_mag = n.subscribe(sub_mag_topic, 10, sub_mag_callback);
	//sub_gps = n.subscribe(sub_gps_topic, 10, sub_gps_callback);
	debug_pub = n.advertise<sensor_msgs::Imu>("/pose_estimate/imu", 10);

	ROS_INFO_STREAM("sub_imu_topic:   "<<sub_imu_topic);
	ROS_INFO_STREAM("sub_mag_topic:   "<<sub_mag_topic);
	ROS_INFO_STREAM("sub_gps_topic:   "<<sub_gps_topic);  
	ROS_INFO_STREAM("use_mag:   "<<use_mag); 

	int hz =200;
	ros::Rate loop_rate(hz);

	IMU::EKF_AHRS ekf_ahrs;
	IMU::Mahony_AHRS mahony_ahrs(Eigen::Vector2d(20.0, 0.005), 0.005);
	IMU::Madgwick_AHRS madgwick_ahrs(100, 0.1);
	t_last = chrono::steady_clock::now(); 
	t_now = chrono::steady_clock::now(); 
    while(ros::ok())
    { 
		 if(use_mag == true)
		 {
			if((imu.IMUFlag&0x01) && (imu.IMUFlag&0x02) && (imu.IMUFlag&0x04))
			{
				unique_lock<mutex> lck(ekfMutex);
				imu.IMUFlag = 0x00;
				//imu.IMUFlag |=0x04;
				 
				Eigen::Vector3d gyro = Eigen::Vector3d(imu.gx,imu.gy,imu.gz);
				Eigen::Vector3d acc = Eigen::Vector3d(imu.ax,imu.ay,imu.az);
				Eigen::Vector3d mag = Eigen::Vector3d(imu.my,imu.mx,imu.mz);
	  			
				double dt =0.005;
			  	t_now = chrono::steady_clock::now(); 
				chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t_now-t_last ); 
				dt = time_used.count();
				if(dt > 0.015)
					dt=0.015;
				//cout<<" cost time: "<<dt<<" s ."<<endl; 
				t_last = t_now; 

				//ekf_ahrs.Run(dt,gyro,acc,mag);
				mahony_ahrs.Run(dt,gyro,acc,mag);
				//madgwick_ahrs.Run(dt,gyro,acc,mag);
			} 
		 }
		else
		{
			if((imu.IMUFlag&0x01) && (imu.IMUFlag&0x02))
			{
				unique_lock<mutex> lck(ekfMutex);
				imu.IMUFlag = 0x00;
			  
				Eigen::Vector3d gyro = Eigen::Vector3d(imu.gx,imu.gy,imu.gz);
				Eigen::Vector3d acc = Eigen::Vector3d(imu.ax,imu.ay,imu.az);
				
				double dt =0.005;
			  	t_now = chrono::steady_clock::now(); 
				chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t_now-t_last ); 
				dt = time_used.count();
				if(dt > 0.015)
					dt=0.015;
				 
				mahony_ahrs.Run(dt,gyro,acc);					
			}
		}

		Eigen::Vector4d dq_imu  = mahony_ahrs.getQuaternion();
		sensor_msgs::Imu debug_msg;
		
		ros::Time times_now = ros::Time::now();
		seq_counter++;
		debug_msg.header.seq = seq_counter;
		debug_msg.header.stamp = times_now;
		debug_msg.header.frame_id = "/imu";

		debug_msg.orientation.x = dq_imu(0);
		debug_msg.orientation.y = dq_imu(1);
		debug_msg.orientation.z = dq_imu(2);
		debug_msg.orientation.w = dq_imu(3);
		debug_msg.orientation_covariance [0] = 0.05;
		debug_msg.orientation_covariance [4] = 0.05;
		debug_msg.orientation_covariance [8] = 0.05;

		debug_msg.angular_velocity.x = imu.gx;
		debug_msg.angular_velocity.y = imu.gy;
		debug_msg.angular_velocity.z = imu.gz;
		debug_msg.linear_acceleration.x = imu.ax;
		debug_msg.linear_acceleration.y = imu.ay;
		debug_msg.linear_acceleration.z = imu.az;
		debug_pub.publish(debug_msg);

		tf::TransformBroadcaster broadcaster;
		broadcaster.sendTransform(tf::StampedTransform(
        	tf::Transform(tf::Quaternion(dq_imu(0), dq_imu(1), dq_imu(2), dq_imu(3)), tf::Vector3(0, 0, 0.5)),
        	ros::Time::now(),"world", "/imu"));	 
		 
        
		ros::spinOnce();
        loop_rate.sleep();
			
    }
   
    std::cout<<" EXIT ..."<<std::endl;
    ros::waitForShutdown();
    ros::shutdown();

    return 1;

}
 
void sub_imu_callback(const sensor_msgs::Imu::ConstPtr imu_msg)
{
	//ROS_INFO_STREAM("recived imu msg ...");
        unique_lock<mutex> lck(imuMutex);
    
	imu.gx = imu_msg->angular_velocity.x;
	imu.gy = imu_msg->angular_velocity.y;
	imu.gz = imu_msg->angular_velocity.z;

	imu.ax = imu_msg->linear_acceleration.x;
	imu.ay = imu_msg->linear_acceleration.y;
	imu.az = imu_msg->linear_acceleration.z;

	imu.qw = imu_msg->orientation.w;
	imu.qx = imu_msg->orientation.x;
	imu.qy = imu_msg->orientation.y;
	imu.qz = imu_msg->orientation.z;
		
	imu.IMUFlag |=0x01;
	imu.IMUFlag |=0x02;
}
void sub_mag_callback(const sensor_msgs::MagneticField::ConstPtr mag_msg)
{
	//ROS_INFO_STREAM("recived mag msg ...");
	unique_lock<mutex> lck(magMutex);

	imu.mx = mag_msg->magnetic_field.x;
	imu.my = mag_msg->magnetic_field.y;
	imu.mz = mag_msg->magnetic_field.z;
	imu.IMUFlag |=0x04;
}
void sub_gps_callback(const sensor_msgs::NavSatFix::ConstPtr gps_msg)
{
	//ROS_INFO_STREAM("recived gps msg ...");
	unique_lock<mutex> lck(gpsMutex);

	imu.GPSLat = gps_msg->latitude;
	imu.GPSLon = gps_msg->longitude;
	imu.high = gps_msg->altitude;
	imu.IMUFlag |=0x08;
} 
// void ExtendKalmanFilter(const double& dt,const Eigen::Vector3d& gyro,
// 			const Eigen::Vector3d& acc,const Eigen::Vector3d& mag) 
// {
  
// 	Eigen::Vector3d acc_m = acc;
// 	acc_m.normalize();
// 	Eigen::Vector3d mag_m = mag; 
// 	mag_m.normalize();


// 	double gyro_norm = gyro.norm();
// 	Eigen::Matrix4d Omega = Eigen::Matrix4d::Zero();
// 	Omega.block<3, 3>(0, 0) = -skewSymmetric(gyro);
// 	Omega.block<3, 1>(0, 3) = gyro;
// 	Omega.block<1, 3>(3, 0) = -gyro;

//   // Some pre-calculation
// //   Eigen::Vector4d dq_dt, dq_dt2;
// //   if (gyro_norm > 1e-5) 
// //   {
// //     dq_dt = (cos(gyro_norm*dt*0.5)*Matrix4d::Identity() + 1/gyro_norm*sin(gyro_norm*dt*0.5)*Omega) * q;
// //     dq_dt2 = (cos(gyro_norm*dt*0.25)*Matrix4d::Identity() + 1/gyro_norm*sin(gyro_norm*dt*0.25)*Omega) * q;
// //   }
// //   else 
// //   {
// //     dq_dt = (Matrix4d::Identity()+0.5*dt*Omega) * cos(gyro_norm*dt*0.5) * q;
// //     dq_dt2 = (Matrix4d::Identity()+0.25*dt*Omega) * cos(gyro_norm*dt*0.25) * q;
// //   }
   
 
// 	//1. F 
// 	//Compute discrete transition and noise covariance matrix
// 	Matrix<double, 7, 7> F = Matrix<double, 7, 7>::Zero();
// 	F.block<4, 4>(0, 0) = 0.5*Omega;

// 	Matrix<double, 7, 7> Fdt = F * dt;
// 	Matrix<double, 7, 7> Fdt_square = Fdt * Fdt;
// 	Matrix<double, 7, 7> Fdt_cube = Fdt_square * Fdt;
// 	Matrix<double, 7, 7> Phi = Matrix<double, 7, 7>::Identity() +
// 		Fdt + 0.5*Fdt_square + (1.0/6.0)*Fdt_cube;

//   //Eigen::Matrix<double, 7, 7> Phi = Eigen::Matrix<double, 7, 7>::Identity();
//   //Phi.block<4, 4>(0, 0) = Eigen::Matrix<double, 4, 4>::Identity()+Omega*0.5*dt;


//   X_IMU = Phi*X_IMU;

//   Eigen::Vector4d qk= X_IMU.head<4>();
//   qk.normalize();
//   X_IMU.head<4>() = qk;
  
//   P = Phi*P*Phi.transpose()+Q;

// 	Eigen::Matrix<double, 3, 3> R_bw = quaternionToRotation(q);


//   //Eigen::Vector3d r_thin = acc_m - R_bw* Eigen::Vector3d(0, 0, 1);
//   //Eigen::Vector3d r_thin= skewSymmetric(z)*quaternionToRotation(q)* Eigen::Vector3d(0, 0, 1);
//   //Eigen::Vector3d r_temp = z - 2*Eigen::Vector3d(q(0)*q(2)-q(1)*q(3), q(3)*q(0)+q(1)*q(2),0.5-q(0)*q(0)-q(1)*q(1));
//   //cout<< "r_thin: " << r_thin.transpose()<<endl;
//   //cout<< "r_temp: " << r_temp.transpose()<<endl;

// 	Eigen::Matrix<double, 3, 7> H_acc ;
// 	H_acc <<  2*q(2), -2*q(3), 2*q(0), -2*q(1), 0, 0, 0,
// 				2*q(3), 2*q(2), 2*q(1), 2*q(0), 0, 0, 0,
// 			-2*q(0), -2*q(1), 2*q(2), 2*q(3), 0, 0, 0;
// 	//这个地方还是应该改为X方向
// 	// Eigen::Vector3d mag_h = R_bw.transpose()*mag_m;
// 	// Eigen::Vector3d mag_b = Eigen::Vector3d( 0, sqrt(mag_h(0)*mag_h(0)+mag_h(1)*mag_h(1)),mag_h(2));
// 	// //mag_b = R_bw* mag_b;
// 	// Eigen::Matrix<double, 3, 7> H_mag ;
// 	// H_mag <<  2*(mag_b(1)*q(1)+mag_b(2)*q(2)), 2*(mag_b(1)*q(0)-mag_b(2)*q(3)), 2*(mag_b(1)*q(3)+mag_b(2)*q(0)), 2*(mag_b(1)*q(2)-mag_b(2)*q(1)), 0, 0, 0,
// 	// 		2*(-mag_b(1)*q(0)+mag_b(2)*q(3)), 2*(mag_b(1)*q(1)+mag_b(2)*q(2)), 2*(-mag_b(1)*q(2)+mag_b(2)*q(1)), 2*(mag_b(1)*q(3)+mag_b(2)*q(0)), 0, 0, 0,
// 	// 		2*(-mag_b(1)*q(3)-mag_b(2)*q(0)), 2*(mag_b(1)*q(2)-mag_b(2)*q(1)), 2*(mag_b(1)*q(1)+mag_b(2)*q(2)), 2*(-mag_b(1)*q(0)+mag_b(2)*q(3)), 0, 0, 0;

// 	 Eigen::Vector3d mag_h = R_bw.transpose()*mag_m;
// 	 Eigen::Vector3d mag_b = Eigen::Vector3d(sqrt(mag_h(0)*mag_h(0)+mag_h(1)*mag_h(1)),0,mag_h(2));
 
// 	 Eigen::Matrix<double, 3, 7> H_mag ;
// 	 H_mag <<  2*(mag_b(0)*q(0)+mag_b(2)*q(2)), 2*(-mag_b(0)*q(1)-mag_b(2)*q(3)), 2*(-mag_b(0)*q(2)+mag_b(2)*q(0)), 2*(mag_b(0)*q(3)-mag_b(2)*q(1)), 0, 0, 0,
// 	 		2*(mag_b(0)*q(1)+mag_b(2)*q(3)), 2*(mag_b(0)*q(0)+mag_b(2)*q(2)), 2*(-mag_b(0)*q(3)+mag_b(2)*q(1)), 2*(-mag_b(0)*q(2)+mag_b(2)*q(0)), 0, 0, 0,
// 	 		2*(mag_b(0)*q(2)-mag_b(2)*q(0)), 2*(mag_b(0)*q(3)-mag_b(2)*q(1)), 2*(mag_b(0)*q(0)+mag_b(2)*q(2)), 2*(mag_b(0)*q(1)+mag_b(2)*q(3)), 0, 0, 0;


// 	Eigen::Matrix<double, 6, 7> H_thin;
// 	H_thin.block<3, 7>(0, 0) = H_acc;
// 	H_thin.block<3, 7>(3, 0) = H_mag;



// 	Eigen::Matrix<double, 6, 1> r_thin;
// 	r_thin.block<3,1>(0,0)= acc_m - R_bw* Eigen::Vector3d(0, 0, 1);
// 	r_thin.block<3,1>(3,0)= mag_m - R_bw*mag_b;

 


// 	Eigen::MatrixXd S = H_thin*P*H_thin.transpose() + R;
// 	Eigen::MatrixXd K = P*H_thin.transpose()*S.inverse();

// 	// Compute the error of the state.
// 	X_IMU = X_IMU+ K * r_thin;


// 	Eigen::Vector4d dq_imu = X_IMU.head<4>();
// 	dq_imu.normalize();
// 	X_IMU.head<4>() = dq_imu;
// 	q = dq_imu;
// 	//cout<< "delta_x: " << delta_x.transpose()<<endl;
// 	//cout<< "q update: " << q.transpose()<<endl;
// 	//cout<< "q: " << q.transpose()<<endl;

// 	P = (Matrix<double, 7, 7>::Identity()-K*H_thin)*P;
// 	P= (P +P.transpose()) / 2.0;


// 	// Eigen::Vector3d out = Quaternion_to_Euler(q).transpose();
// 	// cout<< "Mahony1: roll "<<out(0)<<" pitch:  "<<out(1)<<" yaw:  "<<out(2)<<endl;

// 	quaternionToEuler(q);
//  // ---------show data ----------//
  
// 	sensor_msgs::Imu debug_msg;
// 	debug_msg.header.stamp = ros::Time::now();
// 	debug_msg.header.frame_id = "/imu";

// 	Eigen::Vector3d ea0(imu.roll * M_PI / 180.0,
// 		  imu.pitch * M_PI / 180.0,
// 		  imu.yaw * M_PI / 180.0);
// 	Eigen::Matrix3d R;
// 	R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
// 		* Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
// 		* Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());

// 	Eigen::Quaterniond q_tem;
// 	q_tem = R;
// 	q_tem.normalize();
// 	//q_tem.w() = X_IMU(3);
// 	//q_tem.x() = X_IMU(0);
// 	//q_tem.y() = X_IMU(1);
// 	//q_tem.z() = X_IMU(2);

	 
// 	debug_msg.orientation.w = (double)q_tem.w();
// 	debug_msg.orientation.x = (double)q_tem.x();
// 	debug_msg.orientation.y = (double)q_tem.y();
// 	debug_msg.orientation.z = (double)q_tem.z();
// 	// debug_msg.orientation.w = (double)q(3);
// 	// debug_msg.orientation.x = (double)q(0);
// 	// debug_msg.orientation.y = (double)q(1);
// 	// debug_msg.orientation.z = (double)q(2);
   
// 	debug_msg.angular_velocity.x = gyro(0);
// 	debug_msg.angular_velocity.y = gyro(1);
// 	debug_msg.angular_velocity.z = gyro(2);
// 	debug_msg.linear_acceleration.x = acc(0);
// 	debug_msg.linear_acceleration.y = acc(1);
// 	debug_msg.linear_acceleration.z = acc(2);
// 	debug_pub.publish(debug_msg);

//   //--------------------------------------------------------

//   return;
// }
