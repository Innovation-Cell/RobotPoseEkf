#include "ros/ros.h"
#include "ros/callback_queue.h"

#include "math.h"

#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

using namespace std;


///Variable Declarations
//Used Variables
static float w_roboteq = 0, w_vectornav = 0;
static float VN_WEIGHT_W = 0.1;
static float v = 0, w = 0;
//Flags
static bool vn100_called = 0, roboteq_called = 0;
//To Be Published
nav_msgs::Odometry output;



///Functions for Filter Computation
void complementary(){
	w = VN_WEIGHT_W*w_vectornav + (1 - VN_WEIGHT_W)*w_roboteq;
}

void predict_mean(){
	output.twist.twist.linear.x = v;
	output.twist.twist.angular.z = w;
}

///Sensor Callback Functions
void imuCallback(const sensor_msgs::Imu::ConstPtr& info){
	static double times[2];
	times[0] = times[1];
	times[1] = info->header.stamp.toSec();
	times[0] = times[1] - times[0];

	//Angular
	w_vectornav = info->angular_velocity.z;
	
	//Orientation
	output.pose.pose.orientation.x = info->orientation.x;
	output.pose.pose.orientation.y = info->orientation.y;
	output.pose.pose.orientation.z = info->orientation.z;
	output.pose.pose.orientation.w = info->orientation.w;

	vn100_called = 1;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& info){	
	//Linear
	v = info->twist.twist.linear.x;
	
	//Angular
	w_roboteq = info->twist.twist.angular.z;
	
	roboteq_called = 1;
}



int main(int argc, char* argv[]){
	ros::init(argc, argv, "Velocity_Predictor");
	ros::NodeHandle n;
	
	ros::CallbackQueue sensor_callbacks; 
	ros::CallbackQueueInterface* sensor_callbacks_interface = &sensor_callbacks;
	n.setCallbackQueue(sensor_callbacks_interface);
	
	ros::Subscriber imuMonitor = n.subscribe("/imu_data", 5, imuCallback);
	ros::Subscriber odometryMonitor = n.subscribe("/encoder_data", 5, odometryCallback);
	
	///Publisher for Predict Phase Output
	ros::Publisher publisher = n.advertise<nav_msgs::Odometry>("/velocity", 1);
	
	///Initialization and Fixed Output Data
	output.header.frame_id = "odom_combined";
	output.child_frame_id = "base_footprint";
	output.twist.twist.linear.x = 0;
		
	ros::Rate loop_rate(50);
	output.header.stamp = ros::Time::now();
	while(ros::ok())
	{
		vn100_called = 0; roboteq_called = 0;
		sensor_callbacks.callAvailable();
		complementary();

		predict_mean();

		output.header.stamp = ros::Time::now();
		publisher.publish(output);
		loop_rate.sleep();
	}
	
	return 0;
}
