#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include <time.h>
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"
#include <math.h>
#include "vectornav.h"
#include "std_msgs/Float64MultiArray.h"

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
	Vn200 vn200;
	sensor_msgs::NavSatFix gps_msg;
	geometry_msgs::TwistWithCovarianceStamped nedvel_msg;
	//std_msgs::Float64MultiArray test_msg;
	double gps_time;
	unsigned short gps_week,status;
	VnVector3 ypr,lla,ned_velocity;
	float a_uncertain;
	float vel_uncertain;
	const unsigned int data_sz = 9;
	float pos_uncertain=0;;
	gps_msg.position_covariance_type=3;
/*	test_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	//test_msg.layout.data_offset=0;
	test_msg.layout.dim[0].label="pos_covariance";
	test_msg.layout.dim[0].size=9;
	test_msg.layout.dim[0].stride=1;
	test_msg.data.resize(data_sz);*/
	ros::init(argc,argv,"vn200_data_imu_gps");
	ros::NodeHandle vn;
	ros::Rate loop_rate(20);
	ros::Publisher gps = vn.advertise<sensor_msgs::NavSatFix>("/vn200_insdata_lla",1);
	ros::Publisher ned = vn.advertise<geometry_msgs::TwistWithCovarianceStamped>("/vn200_nedvel",1);
//	ros::Publisher test =vn.advertise<std_msgs::Float64MultiArray>("/test",1);
	
	int connect=0;
	connect=vn200_connect(&vn200,COM_PORT,BAUD_RATE);
	while(connect!=0)
	{
		ROS_ERROR("Connection Failed");
		usleep(999999);
		connect=vn200_connect(&vn200,COM_PORT,BAUD_RATE);
	}
	ROS_INFO("Connected Successfully");
	
	while(ros::ok())
	{
		if(vn200.isConnected)
		{
			vn200_getInsSolution(&vn200, &gps_time, &gps_week, &status, &ypr, &lla, &ned_velocity, &a_uncertain,&pos_uncertain,&vel_uncertain);
		}
		nedvel_msg.header.stamp = ros::Time::now();
		nedvel_msg.header.seq++;
		nedvel_msg.header.frame_id="/odom";
		nedvel_msg.twist.twist.linear = convert_vnvector3_to_vector3(ned_velocity);
		nedvel_msg.twist.covariance[0]=nedvel_msg.twist.covariance[7]=nedvel_msg.twist.covariance[14]=vel_uncertain;
		gps_msg.header.stamp=ros::Time::now();
		gps_msg.header.seq++;
		gps_msg.latitude = lla.c0;
		gps_msg.longitude = lla.c1;
		gps_msg.altitude = lla.c2;
		
			gps_msg.position_covariance[4]=gps_msg.position_covariance[0]=pos_uncertain;
//			test_msg.data[i]=vel_uncertain[i];

		
		gps.publish(gps_msg);
		ned.publish(nedvel_msg);
//		test.publish(test_msg);
		loop_rate.sleep();
	}
	return 0;
}
