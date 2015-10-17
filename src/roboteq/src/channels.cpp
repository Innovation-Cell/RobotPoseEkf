#include "ros/ros.h"

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

using namespace std;


RoboteqDevice device;
int status=0;


int main(int argc, char* argv[]){	
	
	///ROS Initializations
	ros::init(argc, argv, "Roboteq_Channel_Tests");
	ros::NodeHandle n;
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
	
	
	//BETTER TO DO THIS THROUGH ROBORUN....
	///ROBOTEQ closed loop speed settings
	//status = device.SetConfig(_MXMD, 1);
	int mixed_mode;
	device.GetConfig(_MXMD, mixed_mode);
	ROS_INFO("\n\n------Running in mixed mode %d--------\n\n", mixed_mode);
	device.GetConfig(_MMOD, 1, mixed_mode);
	ROS_INFO("\n\n------Motor Channel 1 In Mode %d----------\n\n", mixed_mode);
	device.GetConfig(_MMOD, 2, mixed_mode);
	ROS_INFO("\n\n------Motor Channel 2 In Mode %d----------\n\n", mixed_mode);
	//if(status!=RQ_SUCCESS) ROS_INFO("\n\n----Failed to start mixed mode 1---\nError Code: %d\n", status);
	//else ROS_INFO("\n\n----Device running in closed mixed mode 1----\nError Code: %d\n", status);
	//status=device.SetConfig(_MMOD, 1);
	//if(status!=RQ_SUCCESS) ROS_INFO("\n\n----Failed to start closed position speed mode---\nError Code: %d\n", status);
	//else ROS_INFO("\n\n----Device running in closed loop position mode----\nError Code: %d\n", status);
	
	

	///The Testing
	int ch1_encoder_value=0, ch2_encoder_value=0;
	device.SetCommand(_C, 1, 0);
	device.SetCommand(_C, 2, 0);
	int _max = 0;
	device.SetConfig(_MXRPM, 1, 5000);
	device.SetConfig(_MXRPM, 2, 5000);
	device.SetConfig(_MXMD, 1);
	device.GetConfig(_MXMD, _max);
	ROS_INFO("\n\nNew mixed mode: %d\n\n", _max); 
	device.GetConfig(_MXRPM, 1, _max); //The max RPM value (at motor command 1000 in open/closed-loop speed)
	ROS_INFO("Max RPM 1: %d", _max);
	device.GetConfig(_MXRPM, 2, _max); //The max RPM value (at motor command 1000 in open/closed-loop speed)
	ROS_INFO("Max RPM 2: %d", _max);
	
	//float max = _max;
	//ROS_INFO("Max RPM: %f", max);

	while(ros::ok()){
		device.SetCommand(_G, 2, 50);
//		device.SetCommand(_G, 2, (int)(max/2));
		
//		device.GetValue(_C, 1, ch1_encoder_value);
//		device.GetValue(_C, 2, ch2_encoder_value);
//		ROS_INFO("\nChannel 1 Encoder: %d\nChannel 2 Encoder: %d\n\n", ch1_encoder_value, ch2_encoder_value);
		
		//usleep(100);
	}

	return 0;
}

