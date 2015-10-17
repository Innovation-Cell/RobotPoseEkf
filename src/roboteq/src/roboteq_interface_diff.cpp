#include "ros/ros.h"

#include <time.h>
#include <math.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

#include "roboteq/roboteq_msg.h"

using namespace std;

//Mathematical constants
const float RAD_TO_DEG = 180/3.141592;
const float DEG_TO_RAD = 3.141592/180;

//Vehicle constants
const float Lr = 0.62; //distance between rear wheels (in meters)
const float CIRC = 3.141592*0.38; //wheel circumference (in meters)
const float RED = 66.0;

//User input (w0 in deg/s and v0 in m/s)
const float w0 = 30; //in degrees per second
const float v0 = 0; //in m/s
const int turn_direction = -1; //-1 is right, +1 is left.
const int power_direction = 1; //-1 is back, +1 is forward.

RoboteqDevice device;
roboteq::roboteq_msg output;
const float CUTOFF = 2.0; //actually inverse of required cutoff frequency
int status=0;

//low_pass
void low_pass(roboteq::roboteq_msg &low_f){
	static float rpms[2];
	
	static double times[2];
	times[0] = times[1];
	times[1] = low_f.header.stamp.toSec();
	times[0] = times[1] - times[0];
	 
	ROS_INFO("dt: %f", times[0]);
	
	output.header.stamp = low_f.header.stamp;
	output.header.seq++;

	static float weight; 
	weight = (times[0])/(times[0] + CUTOFF);
	ROS_INFO("Weight: %f", weight);
	
	ROS_INFO("RPM 1: %d, RPM 2: %d", low_f.rpm_1, low_f.rpm_2);
	
	rpms[0] = (weight*((float)low_f.rpm_1)) + ((1-weight)*((float)output.rpm_1));
	rpms[1] = (weight*((float)low_f.rpm_2)) + ((1-weight)*((float)output.rpm_2));

	ROS_INFO("rpms_1: %f, rpms_2: %f", rpms[0], rpms[1]);

	output.rpm_1 = rpms[0]; 
	output.rpm_2 = rpms[1];
	output.encoder_1 = low_f.encoder_1;
	output.encoder_2 = low_f.encoder_2;

	if(output.rpm_1<2&&output.rpm_1>(-2)) output.rpm_1 = 0;
	if(output.rpm_2<2&&output.rpm_2>(-2)) output.rpm_2 = 0;
}
/*
//or differentiate encoders
void diff_enc(roboteq::roboteq_msg &input){
	static float rpms[2];

	static double times[2];	
	times[0] = times[1];
	times[1] = input.header.stamp.toSec();
	times[0] = times[1] - times[0];

	
	ROS_INFO("dt: %F", times[1]);
	
	static int encoders[4];
	encoders[0] = encoders[1]; encoders[2] = encoders[3];
	encoders[1] = input.encoder_1; encoders[3] = input.encoder_2;
	encoders[0] = encoders[1] - encoders[0]; encoders[2] = encoders[3] - encoders[2];
	
	ROS_INFO("Encoder Diff 1: %d, Encoder Diff 2: %d", encoders[0], encoders[2]);
	
	rpms[0] = encoders[0]/times[0]; rpms[1] = encoders[2]/times[0];
	
	output.rpm_1 = (int)rpms[0];
	output.rpm_2 = (int)rpms[1];
	output.encoder_1 = input.encoder_1;
	output.encoder_2 = input.encoder_2;
}
*/
int main(int argc, char* argv[]){	
	
	///ROS Initializations
	ros::init(argc, argv, "Roboteq_Channel_Tests");
	ros::NodeHandle n;
	ros::Publisher publisher = n.advertise<roboteq::roboteq_msg>("/roboteq_monitor", 1);
	ros::Rate loop_rate(20);
	
	
	
	///ROBOTEQ Connection
	ROS_INFO("\n\n--- Roboteq Motor Controller Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	usleep(500000);

	status = device.Connect("/dev/ttyACM1");

	while (status != RQ_SUCCESS && ros::ok())
	{
		ROS_INFO("Error connecting to device: %d\n", status);
		ROS_INFO("Attempting server restart...");
		usleep(999999);
		device.Disconnect();

		status = device.Connect("/dev/ttyAMC1");
		if (status == 0) {
			ROS_INFO("Connection re-established...");
		}
	}
	
	

	///The Publishing Loop
	int ch1_encoder_value = 0, ch2_encoder_value = 0;
	int ch1_rpm_value = 0, ch2_rpm_value = 0;
	roboteq::roboteq_msg roboteqq;
	device.SetCommand(_C, 1, 0);
	device.SetCommand(_C, 2, 0);
	int mode=0;
	device.GetConfig(_MXMD, mode);

	// for (int i=0;i<500;i++){
	// 	device.SetCommand(_G, 1, 500); device.SetCommand(_G, 2, 500);
	// }

	while(ros::ok()){
		/*Gathering Data*/
			//Directly from Device
			device.GetValue(_S, 1, ch1_rpm_value);
			device.GetValue(_S, 2, ch2_rpm_value);
			device.GetValue(_C, 1, ch1_encoder_value);
			device.GetValue(_C, 2, ch2_encoder_value);
			device.SetCommand(_G, 1, 500); 
			device.SetCommand(_G, 2, 500);
			int max;
			device.GetConfig(_MXRPM, max); //max rpm value goes into the message simulatenously
			roboteqq.mxrpm = max;
			//Debugging
			ROS_INFO("\nChannel 1 Encoder: %d  Channel 2 Encoder: %d", ch1_encoder_value, ch2_encoder_value);
			ROS_INFO("Channel 1 RPM: %d  Channel 2 RPM: %d\n", ch1_rpm_value, ch2_rpm_value);
	
		/*Filling in Data*/
			//Time
			roboteqq.header.stamp = ros::Time::now();
			//RPM Stuff
			roboteqq.rpm_1 = ch1_rpm_value; roboteqq.rpm_2 = ch2_rpm_value;
			//Encoder Stuff
			roboteqq.encoder_1 = ch1_encoder_value; roboteqq.encoder_2 = ch2_encoder_value;

		low_pass(roboteqq);
		//diff_enc(roboteqq);

		/*Publishing*/
		publisher.publish(output);
		loop_rate.sleep();
	}
	
	return 0;
}

