#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "geodesy/utm.h"

#include "math.h"

#include "nav_msgs/Odometry.h"

using namespace std;


///Variable Declarations
//Mathematical Constants
const float RAD_TO_DEG = 180/3.141592;
//Used Variables
static float x_increment = 0, y_increment = 0;
//Flags
static bool velocity_called = 0;
//To Be Published
static nav_msgs::Odometry output;
//Output Variables
double linearVelocityVariance = 0.01;
double angularVelocityVariance = 0.01;


///Functions for Filter Computation
void predict_mean()
{	
	output.pose.pose.position.x += x_increment;
	output.pose.pose.position.y -= y_increment;	
}


///Callback Functions
void Velocity_Callback(const nav_msgs::Odometry::ConstPtr& info)
{		
	static double increment = 0, heading = 0;
	
	if(info->twist.twist.linear.x!=0){
		//Timestamp stuff
		static double times[2];
		times[0] = times[1];
		times[1] = info->header.stamp.toSec();
		times[0] = times[1] - times[0];
		
		//Orientation
		output.pose.pose.orientation.x = info->pose.pose.orientation.x;
		output.pose.pose.orientation.y = info->pose.pose.orientation.y;
		output.pose.pose.orientation.z = info->pose.pose.orientation.z;
		output.pose.pose.orientation.w = info->pose.pose.orientation.w;
		heading = tf::getYaw(info->pose.pose.orientation);
		//ROS_INFO("Heading: %f", heading*RAD_TO_DEG);

		//Twist
		output.twist.twist.linear.x = (info->twist.twist.linear.x)*(cos(heading));
		output.twist.twist.linear.y = (info->twist.twist.linear.x)*(sin(heading));
		output.twist.twist.angular.z = info->twist.twist.angular.z;
		
		//Position
		increment = (info->twist.twist.linear.x)*times[0];
		if(increment<10&&increment>-10){
			x_increment += increment*(cos(heading));
			y_increment += increment*(sin(heading));
		}
		else {
			x_increment = 0;
			y_increment = 0;
		}
		ROS_INFO("x_increment: %f", x_increment);
		velocity_called = 1;
	}
}



int main(int argc, char* argv[])
{
	ros::init(argc, argv, "position_localization");
	ros::NodeHandle n;
	
	ros::CallbackQueue sensor_callbacks; 
	ros::CallbackQueueInterface* sensor_callbacks_interface = &sensor_callbacks;
	n.setCallbackQueue(sensor_callbacks_interface);

	ros::Subscriber velocityMonitor = n.subscribe("/velocity", 5, Velocity_Callback);
	
	
	///Publisher for Predict Phase Output
	ros::Publisher publisher = n.advertise<nav_msgs::Odometry>("/odom", 1);
	
	
	///Initializing and Filling in Fixed Data
	output.pose.pose.position.x=0; output.pose.pose.position.y=0; output.pose.pose.position.z=0;
	output.pose.pose.orientation.x=0; output.pose.pose.orientation.y=0; output.pose.pose.orientation.z=0; output.pose.pose.orientation.w=1;
	for(int i=0; i<36; i++){
		if((i/6)==(i%6)) output.pose.covariance[i]=99999;
		else output.pose.covariance[i]=0;
	}
	output.twist.twist.linear.x = 0; output.twist.twist.linear.y = 0; output.twist.twist.linear.z = 0;
	output.twist.twist.angular.x = 0; output.twist.twist.angular.y = 0; output.twist.twist.angular.y = 0;
	for(int i=0; i<36; i++){
		if((i/6)==(i%6)) output.twist.covariance[i]=99999;
		else output.twist.covariance[i]=0;
	}
	
		
	output.header.frame_id = "odom_combined";
	output.child_frame_id = "base_footprint";
	
	ros::Rate loop_rate(50);
	while(ros::ok())
	{
		velocity_called = 0;
		sensor_callbacks.callAvailable();
		predict_mean();

		if(velocity_called){			
			x_increment = 0; y_increment = 0;
		}
		output.twist.covariance[0] = linearVelocityVariance;
		output.twist.covariance[7]=linearVelocityVariance;
		output.twist.covariance[35] = angularVelocityVariance;
		output.pose.covariance[0] = 0.01;
		output.pose.covariance[7] = 0.01;
		output.pose.covariance[35]=0.01;

		output.header.stamp = ros::Time::now();
		publisher.publish(output);
		loop_rate.sleep();
	}
	
	return 0;
}
