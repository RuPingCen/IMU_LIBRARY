/**
 * @function 读取 Xsens 的数据 利用加速度的观察信息 动态调整 Mahony_AHRS Kp的值 
 *      实现变权重的姿态融合算法，并对比原始的Mahony_AHRS的算法
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
#include "Mahony_AHRS.h"
 

using namespace std;
using namespace Eigen;
using namespace IMU;

std::mutex imuMutex,magMutex,gpsMutex;
std::mutex ekfMutex;

uint32_t seq_counter=0;

ros::Subscriber sub_imu,sub_mag,sub_gps;
ros::Publisher debug_pub,debug_pub2;

enum Algorithm
{
	EKF=1,  
	Mahony,
	Madgwick	
};
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
	string sub_imu_topic,sub_mag_topic,sub_gps_topic,algorithm;
	bool use_mag = false;
	  
	ros::init(argc, argv, "mahony_dynamic_kp_test");
	ros::NodeHandle n("~");


	n.param<std::string>("sub_imu_topic", sub_imu_topic, "/imu/data");
	n.param<std::string>("sub_mag_topic", sub_mag_topic, "/imu/mag");
	n.param<bool>("use_mag", use_mag, true);
	  
	sub_imu = n.subscribe(sub_imu_topic, 10, sub_imu_callback);
	sub_mag = n.subscribe(sub_mag_topic, 10, sub_mag_callback);
	 
	debug_pub = n.advertise<sensor_msgs::Imu>("/pose_estimate/static_kp/imu", 10);
	debug_pub2= n.advertise<sensor_msgs::Imu>("/pose_estimate/dynamic_kp/imu", 10);

	ROS_INFO_STREAM("sub_imu_topic:   "<<sub_imu_topic);
	ROS_INFO_STREAM("sub_mag_topic:   "<<sub_mag_topic);
	ROS_INFO_STREAM("use_mag:   "<<use_mag); 
  
	int hz =200;
	ros::Rate loop_rate(hz);

	IMU::Mahony_AHRS mahony_ahrs_static_kp(Eigen::Vector2d(20, 0.01), 0.005);
	IMU::Mahony_AHRS mahony_ahrs_dynamic_kp(Eigen::Vector2d(20, 0.01), 0.005);

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
 
				mahony_ahrs_static_kp.Run(dt,gyro,acc,mag);
				mahony_ahrs_dynamic_kp.Run2(dt,gyro,acc,mag); //变权重
			  
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

				mahony_ahrs_static_kp.Run(dt,gyro,acc);
				mahony_ahrs_dynamic_kp.Run(dt,gyro,acc);
				
			}
		}

		Eigen::Vector4d dq_imu = mahony_ahrs_static_kp.getQuaternion();
		 
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


		Eigen::Vector4d dq_imu_dynamic_kp= mahony_ahrs_dynamic_kp.getQuaternion();
		 
		sensor_msgs::Imu debug_msg_dynamic_kp;
		 
		debug_msg_dynamic_kp.header.seq = seq_counter;
		debug_msg_dynamic_kp.header.stamp = times_now;
		debug_msg_dynamic_kp.header.frame_id = "/imu";

		debug_msg_dynamic_kp.orientation.x = dq_imu_dynamic_kp(0);
		debug_msg_dynamic_kp.orientation.y = dq_imu_dynamic_kp(1);
		debug_msg_dynamic_kp.orientation.z = dq_imu_dynamic_kp(2);
		debug_msg_dynamic_kp.orientation.w = dq_imu_dynamic_kp(3);
		debug_msg_dynamic_kp.orientation_covariance [0] = 0.05;
		debug_msg_dynamic_kp.orientation_covariance [4] = 0.05;
		debug_msg_dynamic_kp.orientation_covariance [8] = 0.05;

		debug_msg_dynamic_kp.angular_velocity.x = imu.gx;
		debug_msg_dynamic_kp.angular_velocity.y = imu.gy;
		debug_msg_dynamic_kp.angular_velocity.z = imu.gz;
		debug_msg_dynamic_kp.linear_acceleration.x = imu.ax;
		debug_msg_dynamic_kp.linear_acceleration.y = imu.ay;
		debug_msg_dynamic_kp.linear_acceleration.z = imu.az;
		debug_pub2.publish(debug_msg_dynamic_kp);



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
