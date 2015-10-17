#include "ros/ros.h"

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

#include "localization/roboteq_msg.h"
#include "nav_msgs/Odometry.h"


using namespace std;

///Could use motor encoder values and heading wrt true north from /imu topic 
///to give pose estimate (right now this node only outputs a twist estimate).

//Mathematical constants
const float RAD_TO_DEG = 180/3.141592;
const float DEG_TO_RAD = 3.141592/180;

const float CF = 0.9; //Correction Factor

//Vehicle constants
const float Lr = 0.65; //distance between wheels (in meters)
const float CIRC = 3.141592*0.38; //wheel circumference (in meters)
const float REDUCTION = 91.0;
const float RPM_TO_RPS = 1/60.0; //from per minute to per second
const float v_factor = (0.93*2*CIRC*RPM_TO_RPS)/(2*REDUCTION);
const float w_factor = (CF*CIRC*2*RPM_TO_RPS)/(2*Lr*REDUCTION);
			
//Device driver variables
RoboteqDevice device;
int status=0;

//Low-pass filter variables and constants
localization::roboteq_msg filtered;
const float inverseCutoff = 0.5;

//Output variables
nav_msgs::Odometry output;
double v, w; //v in m/s, w in deg/s
double linearVelocityVariance = 0.01;
double angularVelocityVariance = 0.01;

void low_pass(localization::roboteq_msg &unfiltered)
{
	static float rpms[2];
	
	static double times[2];
	times[0] = times[1];
	times[1] = unfiltered.header.stamp.toSec();
	times[0] = times[1] - times[0];

	static float weight; 
	weight = (times[0])/(times[0] + inverseCutoff);
 	
	filtered.header.stamp = unfiltered.header.stamp;
	
	rpms[0] = (weight*((float)unfiltered.rpm_1)) + ((1-weight)*((float)filtered.rpm_1));
	rpms[1] = (weight*((float)unfiltered.rpm_2)) + ((1-weight)*((float)filtered.rpm_2));

	filtered.rpm_1 = rpms[0]; 
	filtered.rpm_2 = rpms[1];

	if(filtered.rpm_1<2&&filtered.rpm_1>(-2)) filtered.rpm_1 = 0;
	if(filtered.rpm_2<2&&filtered.rpm_2>(-2)) filtered.rpm_2 = 0;
}


int main(int argc, char* argv[])
{	
	
	///ROS Initializations
	ros::init(argc, argv, "Roboteq_Channel_Tests");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<nav_msgs::Odometry>("/encoder_data", 1);
	ros::Rate loop_rate(20);
	
	
	
	///ROBOTEQ Connection
	ROS_INFO("\n\n--- Roboteq Motor Controller Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	usleep(500000);

	status = device.Connect("/dev/ttyACM0");

	while (status != RQ_SUCCESS && ros::ok())
	{
		ROS_ERROR("Error connecting to device: %d\n", status);
		ROS_INFO("Attempting server restart...");
		usleep(999999);
		device.Disconnect();

		status = device.Connect("/dev/ttyAMC0");
		if (status == 0) {
			ROS_INFO("Connection re-established...");
		}
	}
	
	///Initialization and Fixed Data
	output.pose.pose.position.x=0; output.pose.pose.position.y=0; output.pose.pose.position.z=0;
	output.pose.pose.orientation.x=1; output.pose.pose.orientation.y=0; output.pose.pose.orientation.z=0; output.pose.pose.orientation.w=0;
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

	///The Publishing Loop
	device.SetCommand(_C, 1, 0);
	device.SetCommand(_C, 2, 0);
	int ch1_rpm_value = 0, ch2_rpm_value = 0;
	localization::roboteq_msg unfiltered;

	output.header.frame_id = "odom_combined";
	output.child_frame_id = "base_footprint";
	
	while(ros::ok()){
		/*Gathering Data from Device*/
			device.GetValue(_S, 1, ch1_rpm_value);
			device.GetValue(_S, 2, ch2_rpm_value);
	
		/*Preparing Data for Low-Pass Filter*/
			//Time
			unfiltered.header.stamp = ros::Time::now();
			//RPM Stuff
			unfiltered.rpm_1 = ch1_rpm_value; unfiltered.rpm_2 = ch2_rpm_value;
		
		/*LowPass Filtering*/
		low_pass(unfiltered);
		
		/*Calculating 'v' and 'w' From Filtered Data*/
			//Calculate 'v'
			v = v_factor*(filtered.rpm_2 - filtered.rpm_1); //rpm_1 is negatived because when the wheel turns forwards, rpm_1 is negative
			//Calculate 'w'
			w = w_factor*(filtered.rpm_2 + filtered.rpm_1); //positive is left, negative is right (rpm_2 refers to right wheel)

		/*Data to Output Message*/
			//Header Data
			output.header.stamp = filtered.header.stamp;
			output.header.seq++;
			//Twist Data
			output.twist.twist.linear.x = v;
			output.twist.twist.angular.z = w;
			//Covariances
			output.twist.covariance[0] = linearVelocityVariance;
			output.twist.covariance[35] = angularVelocityVariance;
		
		/*Publishing*/
		publisher.publish(output);
		loop_rate.sleep();
	}
	
	return 0;
}

