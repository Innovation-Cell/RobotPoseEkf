#include "ros/ros.h"

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
const int turn_direction = 1; //-1 is right, +1 is left.
const int power_direction = 1; //-1 is back, +1 is forward.

RoboteqDevice device;
roboteq::roboteq_msg output;
const float CUTOFF = 2.0; //actually inverse of required cutoff frequency
int status=0;

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

	status = device.Connect("/dev/ttyACM0");

	while (status != RQ_SUCCESS && ros::ok())
	{
		ROS_INFO("Error connecting to device: %d\n", status);
		ROS_INFO("Attempting server restart...");
		usleep(999999);
		device.Disconnect();

		status = device.Connect("/dev/ttyAMC0");
		if (status == 0) {
			ROS_INFO("Connection re-established...");
		}
	}
	
	

	///The Publishing Loop
	int ch1_encoder_value = 0, ch2_encoder_value = 0;
	int ch1_rpm_value = 0, ch2_rpm_value = 0;
	roboteq::roboteq_msg roboteqq;
	device.SetCommand(_C, 1, 0); device.SetCommand(_C, 2, 0);
	device.SetConfig(_MXRPM, 1, 5000); device.SetConfig(_MXRPM, 2, 5000);
	while(ros::ok()){
	while(ros::ok()){
		static float v, w;
		int _max = 0; device.GetConfig(_MXRPM, _max); //The max RPM value (at motor command 1000 in open/closed-loop speed)
		float max = _max;
		ROS_INFO("Max RPM: %f", max);
		//For calculating command to send to channel 2 (turning)
			w = w0*DEG_TO_RAD; //in terms of rad/s.
			w *= Lr/2; //in m/s
			w *= (60/CIRC); //in RPM (direction unspecified)
			w *= (float)turn_direction; //in RPM (direction specified)
			w *= RED;
			ROS_INFO("RPM Command: %f", w);
			w *= (1000/max); //in motor command
			
		//For calculating command to be sent to channel 1 (forward movement)
			v = v0; //in m/s
			v *= (60/CIRC); //in RPM (direction unspecified)
			v *= (float)power_direction; //in RPM (direction specified)
			v *= RED;				
			v *= (1000/max); //in motor command for open or closed-loop speed
			
		//exception handler	
			if((((v+w)*power_direction)>2000)||(((v-w)*power_direction)>2000)){ 
				ROS_INFO("Motor commands required are exceding safe levels.\n Re-calculating command values\n"); 
				device.SetCommand(_G, 1, 0); device.SetCommand(_G, 2, 0);
				//exit(EXIT_FAILURE);}
				break;
			}
			
		//For Issuing Commands.
			int w_final = w;
			int v_final = v;
			
			ROS_INFO("RPMs: %d", (int)(w_final*(max/1000)));
			//ROS_INFO("v_final: %d", v_final);
			//ROS_INFO("w_final: %d", w_final);
			
			device.SetCommand(_G, 2, v_final);
			device.SetCommand(_G, 1, w_final);

		/*Gathering Data*/
			//Directly from Device
			device.GetValue(_S, 1, ch1_rpm_value);
			device.GetValue(_S, 2, ch2_rpm_value);
			device.GetValue(_C, 1, ch1_encoder_value);
			device.GetValue(_C, 2, ch2_encoder_value);
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

		/*Publishing*/
		publisher.publish(output);
		loop_rate.sleep();
	}
	}
	return 0;
}

