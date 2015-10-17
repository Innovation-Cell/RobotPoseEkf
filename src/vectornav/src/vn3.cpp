#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include <stdio.h>
#include <time.h>
#include "vectornav.h"
#include <iostream>

using namespace std;
const char* const COM_PORT = "/dev/ttyUSB0";
const int BAUD_RATE=115200;

geometry_msgs::Vector3 convert_vnvector3_to_vector3(VnVector3 vector)
{
	geometry_msgs::Vector3 v1;
	v1.x=vector.c0;
	v1.y=vector.c1;
	v1.z=vector.c2;
	return v1;
}

int main(int argc,char **argv)
{
	Vn100 vn100;
	VnYpr ypr;
	VnVector3 angvel,acc,mag;
	
	ros::init(argc,argv,"vn100_gpsdata");
	ros::NodeHandle gps;
	ros::Publisher data = gps.advertise<sensor_msgs::Imu>("/vn100_Monitor",1);
	
	int connect=0;
	connect=vn100_connect(&vn100,COM_PORT,BAUD_RATE);
	while(connect!=0)
	{
		ROS_ERROR("Connection Error. Retrying...");
		usleep(999999);
		connect=vn100_connect(&vn100,COM_PORT,BAUD_RATE);
	}	
	ROS_INFO("VN-100 Connected");
	sensor_msgs::Imu message;
	ros::Rate loop_rate(20);
	while(ros::ok())
	{
		if(vn100.isConnected)
		{
			vn100_getYawPitchRollMagneticAccelerationAngularRate(&vn100,&ypr,&mag,&acc,&angvel);
		}else ROS_ERROR("Sensor Disconnected");
		
		message.header.stamp=ros::Time::now();
		message.header.seq++;
		message.header.frame_id="/odom";
		message.orientation.x=ypr.roll;
		message.orientation.y=ypr.pitch;
		message.orientation.z=ypr.yaw;
		message.angular_velocity=convert_vnvector3_to_vector3(angvel);
		message.linear_acceleration=convert_vnvector3_to_vector3(acc);
		data.publish(message);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;

}
