#include "ros/ros.h"

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"

///ROBOTEQ needs to be running in Mixed Mode.

using namespace std;

//Mathematical constants
const float RAD_TO_DEG = 180/3.141592;
const float DEG_TO_RAD = 3.141592/180;

//Vehicle constants
const float Lr = 0.62; //distance between rear wheels (in meters)
const float CIRC = 3.141592*0.38; //wheel circumference (in meters)
const float RED = 66.0;

//User input (w0 in deg/s and v0 in m/s)
const float w0 = 30*(5.0/4.6); //in degrees per second
const float v0 = .25; //in m/s
const int turn_direction = -1; //-1 is right, +1 is left.
const int power_direction = 1; //-1 is back, +1 is forward.

//Rate of changes
float w, v;

//Device code required
int status=0;
RoboteqDevice device;


int main(int argc, char* argv[]){	
	
	///ROS Initializations
		ros::init(argc, argv, "system_model");
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
			//usleep(999999);
			//device.Disconnect();
		
				status = device.Connect("/dev/ttyACM0");
			if (status == 0) {
				ROS_INFO("Connection re-established...");
			}
		}

	
	
	
	///MMOD Debugging
		int mode=0;
		device.GetConfig(_MXMD, mode);
		ROS_INFO("Operating mode: %d", mode);
		ros::Duration(1.0).sleep();
	
	
	///System Model
	float times[2];
	device.SetConfig(_MXRPM, 1, 5000);
	device.SetConfig(_MXRPM, 2, 5000);
	
	while(ros::ok()){
	while(ros::ok()){		
			int _max = 0; device.GetConfig(_MXRPM, _max); //The max RPM value (at motor command 1000 in open/closed-loop speed)
			float max = _max;
			ROS_INFO("Max RPM: %f", max);
		//For calculating command to send to channel 2 (turning)
			w = w0*DEG_TO_RAD; //in terms of rad/s.
			w *= Lr/2; //in m/s
			w *= (60/CIRC); //in RPM (direction unspecified)
			w *= (float)turn_direction; //in RPM (direction specified)
			w *= (1000/max); //in motor command
			w *= RED;
			
		//For calculating command to be sent to channel 1 (forward movement)
			v = v0; //in m/s
			v *= (60/CIRC); //in RPM (direction unspecified)
			v *= (float)power_direction; //in RPM (direction specified)
			v *= (1000/max); //in motor command for open or closed-loop speed
			v *= RED;		
			
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
			
			ROS_INFO("v_final: %d", v_final);
			ROS_INFO("w_final: %d", w_final);
			
			device.SetCommand(_G, 2, v_final);
			device.SetCommand(_G, 1, w_final);
	}
	}

	return 0;
}
	
	
