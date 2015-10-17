#include "ros/ros.h"
#include "tf/tf.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"

#include <iostream>
#include <stdio.h>
#include <math.h>

#include "vectornav.h"

#include "localization/vn100_msg.h"
#include "localization/lp.h"



using namespace std;

const char* const COM_PORT = "/dev/ttyUSB0";
const int BAUD_RATE = 115200;
const float INV_CUTOFF = 1/20.0;
const float RAD_TO_DEG = 180/3.141592;

static sensor_msgs::Imu output;	


void low_pass(localization::lp &unfiltered){
	float dt = unfiltered.stamp.toSec() - output.header.stamp.toSec();

	output.header.stamp = unfiltered.stamp;

	float weight = (dt)/(dt + INV_CUTOFF);

	unfiltered.av_x *= RAD_TO_DEG; unfiltered.av_y *= RAD_TO_DEG; unfiltered.av_z *= RAD_TO_DEG; 
	
	output.linear_acceleration.x = (weight*unfiltered.x) + ((1-weight)*output.linear_acceleration.x);
	output.linear_acceleration.y = (weight*unfiltered.y) + ((1-weight)*output.linear_acceleration.y);
	output.linear_acceleration.z = (weight*unfiltered.z) + ((1-weight)*output.linear_acceleration.z);
	output.angular_velocity.x = (weight*unfiltered.av_x) + ((1-weight)*output.angular_velocity.x);
	output.angular_velocity.y = (weight*unfiltered.av_y) + ((1-weight)*output.angular_velocity.y);
	output.angular_velocity.z = (weight*unfiltered.av_z) + ((1-weight)*output.angular_velocity.z);

	if(output.linear_acceleration.x<0.3&&output.linear_acceleration.x>(-0.3)) output.linear_acceleration.x = 0;
	if(output.linear_acceleration.y<0.3&&output.linear_acceleration.y>(-0.3)) output.linear_acceleration.y = 0;
	if(output.linear_acceleration.z<0.3&&output.linear_acceleration.z>(-0.3)) output.linear_acceleration.z = 0;	
	if(output.angular_velocity.x<0.3&&output.angular_velocity.x>(-0.3)) output.angular_velocity.x = 0;
	if(output.angular_velocity.y<0.3&&output.angular_velocity.y>(-0.3)) output.angular_velocity.y = 0;
	if(output.angular_velocity.z<0.3&&output.angular_velocity.z>(-0.3)) output.angular_velocity.z = 0;

}	



int main(int argc, char **argv)
{
	///Variable Declerations
	Vn100 vn100;
	VnYpr attitude;
	VnVector3 bodyAcceleration, angularRate;
	VnVector3 angularRateVariance, magneticVariance, accelerationVariance;
	double angularWalkVariance;
	
	///ROS Node Initialization
	ros::init(argc, argv, "IMU_Interface"); 
	ros::NodeHandle imu;
	ros::Publisher imuDataPublisher = imu.advertise<sensor_msgs::Imu>("/imu_data", 1);
	ros::Publisher infer = imu.advertise<localization::vn100_msg>("/vn100_monitor", 1);

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
	
	localization::lp unfiltered;
	localization::vn100_msg info;
	output.header.frame_id = "imu";

	while (ros::ok()) {
		//Getting Data//
		if(vn100.isConnected){	
			vn100_getYawPitchRollTrueInertialAccelerationAngularRate(&vn100, &attitude, &bodyAcceleration, &angularRate);
		} else ROS_ERROR("Connection Error...");

		if(vn100.isConnected){	
			vn100_getFilterMeasurementVarianceParameters(&vn100, &angularWalkVariance, &angularRateVariance, &magneticVariance, &accelerationVariance);
		} else ROS_ERROR("Connection Error...");

		//Filling in Data//
			//Data for Low Pass Filter
				//Header data
				unfiltered.stamp = ros::Time::now();
				//IMU data
				unfiltered.x = bodyAcceleration.c0; unfiltered.y = bodyAcceleration.c1; unfiltered.z = bodyAcceleration.c2;
				unfiltered.av_x = angularRate.c0; unfiltered.av_y = angularRate.c1; unfiltered.av_z = angularRate.c2;
				low_pass(unfiltered);

			
			//Data for Publishing
				//Header data
				output.header.seq++;
				//Orientation
				tf::Quaternion rotation;
				rotation.setRPY(attitude.roll*(1/RAD_TO_DEG), attitude.pitch*(1/RAD_TO_DEG), attitude.yaw*(1/RAD_TO_DEG));
				output.orientation.x = (double) rotation.x();
				output.orientation.y = (double) rotation.y();
				output.orientation.z = (double) rotation.z();
				output.orientation.w = (double) rotation.w();
				for (int i=0; i<9; i++){
					output.orientation_covariance[i] = 0;
				}
				output.orientation_covariance[0] = 0.01;
				output.orientation_covariance[4] = 0.01;
				output.orientation_covariance[8] = 0.01;
				//Angular Velocity Variance
				for(int i=0; i<9; i++){
					output.angular_velocity_covariance[i] = 0;
				}
				output.angular_velocity_covariance[0] = angularRateVariance.c0;
				output.angular_velocity_covariance[4] = angularRateVariance.c1;
				output.angular_velocity_covariance[8] = angularRateVariance.c2;
				//Linear Acceleration Variance
				for(int i=0; i<9; i++){
					output.linear_acceleration_covariance[i] = 0;
				}
				output.linear_acceleration_covariance[0] = accelerationVariance.c0;
				output.linear_acceleration_covariance[4] = accelerationVariance.c1;
				output.linear_acceleration_covariance[8] = accelerationVariance.c2;


				//For vn100_monitor (i.e. for running the vehicle along a particular heading)
				info.header.stamp = output.header.stamp; info.header.seq++;
				info.linear.x = output.linear_acceleration.x; info.linear.y = output.linear_acceleration.y;
				info.linear.theta = attitude.yaw;
				info.angular_velocity = output.angular_velocity.z;
		
		//double headingDebugging = tf::getYaw(output.orientation);
		//ROS_INFO("Heading sent: %f", headingDebugging);
		
		//Other Stuff//
		imuDataPublisher.publish(output);
		infer.publish(info);
		loop_rate.sleep(); 

	}

	return 0;

}

