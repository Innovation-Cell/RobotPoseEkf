#include "ros/ros.h"

#include <iostream>
#include <stdio.h>
#include <math.h>

#include "vectornav.h"

#include "vectornav/vn100_msg.h"
#include "vectornav/lp.h"

using namespace std;

const char* const COM_PORT = "/dev/ttyUSB0";
const int BAUD_RATE = 115200;
const float CUTOFF = 1/20.0; //actually the inverse of the required cutoff frequency
const float THRESHOLD = 0.05;
const float RAD_TO_DEG = 180/3.141592;
static vectornav::lp low_pass_filtered;

//Dicretised RC implementation of low pass filter

void low_pass(vectornav::lp &low_f){
	float dt = low_f.stamp.toSec() - low_pass_filtered.stamp.toSec();
//	ROS_INFO("dt: %f", dt);

	low_pass_filtered.stamp = low_f.stamp;

	float weight = (dt)/(dt + CUTOFF);
//	ROS_INFO("Weight: %f", weight);
	
//	ROS_INFO("Angular velocity: %f", low_f.ang_vel);
	low_f.ang_vel *= RAD_TO_DEG;
	
	low_pass_filtered.x = (weight*low_f.x) + ((1-weight)*low_pass_filtered.x);
	low_pass_filtered.y = (weight*low_f.y) + ((1-weight)*low_pass_filtered.y);
	low_pass_filtered.ang_vel = (weight*low_f.ang_vel) + ((1-weight)*low_pass_filtered.ang_vel);	//actually, angular velocity

	if(low_pass_filtered.x<0.3&&low_pass_filtered.x>(-0.3)) low_pass_filtered.x = 0;
	if(low_pass_filtered.y<0.3&&low_pass_filtered.y>(-0.3)) low_pass_filtered.y = 0;
	if(low_pass_filtered.ang_vel<0.3&&low_pass_filtered.ang_vel>(-0.3)) low_pass_filtered.ang_vel = 0;
}	

int main(int argc, char **argv)
{
	///Variable Declerations
	Vn100 vn100;
	VnYpr attitude;
	VnVector3 bodyAcceleration, angularRate;
	
	///ROS Node Initialization
	ros::init(argc, argv, "VN100_Interface"); 
	ros::NodeHandle imu;
	ros::Publisher infer = imu.advertise<vectornav::vn100_msg>("/vn100_monitor", 1);


	///Connection to Vectornav
	int connect = 0;
	ROS_INFO("\n\n--- Vectornav IMU Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	connect = vn100_connect(&vn100, COM_PORT, BAUD_RATE);
	ROS_INFO("Connected ...");
	while(connect != 0){
		ROS_ERROR("Not Connected ...");
		usleep(999999);
 	}
	ROS_INFO("Server initialized...");
	
	
	///Publishing Loops
	ros::Rate loop_rate(20);
	
	vectornav::lp low_f; 
	vectornav::vn100_msg info;
	
	while (ros::ok()) {
		/*Getting Data*/
		if(vn100.isConnected){	
			vn100_getYawPitchRollTrueBodyAccelerationAngularRate(&vn100, &attitude, &bodyAcceleration, &angularRate);
		} else ROS_INFO("Connection Error...");
		
		/*Filling in Data*/
			//Header Data for Low Pass Filter
			low_f.stamp = ros::Time::now();
			//IMU Data for Low Pass Filter
			low_f.x = bodyAcceleration.c0; low_f.y = bodyAcceleration.c1;
			low_f.ang_vel = angularRate.c2;// + sqrt(bodyAcceleration.c0/0.34);
//			ROS_INFO("asfsdfsdf_2: %f", low_f.ang_vel*RAD_TO_DEG);
			//ROS_INFO("\n\nAdditive offset for angular velocity: %f\n", bodyAcceleration.c0*0.34*(180/3.141592));
			//low_f.x = bodyAcceleration.c0 - low_f.ang_vel*low_f.ang_vel*0.34*(180/3.141592)*(180/3.141592);
			//ROS_INFO("\nOffset for removing effect of angular velocity on x-acceleration: %f\n", (180/3.141592)*(180/3.141592)*low_f.ang_vel*low_f.ang_vel*0.34);
			//Low Pass Filter
			low_pass(low_f);

			//Data for Publishing
			info.header.stamp = low_pass_filtered.stamp; info.header.seq++;
			info.linear.x = low_pass_filtered.x; info.linear.y = low_pass_filtered.y;
			info.linear.theta = attitude.yaw;
			info.angular_velocity = low_pass_filtered.ang_vel;
/*			ROS_INFO("Linear Accelerations: ");
			ROS_INFO("x: %f", bodyAcceleration.c0); //ROS_INFO("x: %f", info.linear.x);
			ROS_INFO("y: %f", bodyAcceleration.c1); //ROS_INFO("y: %f", info.linear.y);
			ROS_INFO("Angular Velocity: %f", low_pass_filtered.ang_vel);
			ROS_INFO("Heading: %f\n", info.linear.theta);
*/			
		/*Other Stuff*/
		infer.publish(info);
		loop_rate.sleep(); 

	}

	return 0;

}

