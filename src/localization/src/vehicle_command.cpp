#include "ros/ros.h"
#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"
#include "math.h"


using namespace std;

const float RAD_TO_DEG = 180/3.141592;
const float DEG_TO_RAD = 3.141592/180;

const float Lr = 0.65;       
const float CIRC = 3.141592*0.38; 
const float RED = 91.0;

float v = 0.1; 				// in m/s
float w = 0; 				// in rad/s

float v0, w0;

const float correction_factor = 1.0;

int status=0;
RoboteqDevice device;

void motorcommand()
{
	int _max = 0; 
	device.GetConfig(_MXRPM, _max); 
	float max = _max;
	
	w0 = -(Lr*RED*w)/(CIRC*(1/60.0)); 
	
	v0 = -(2*RED*v)/(CIRC*(1/60.0));     		
			
	if((v0+w0)>2000 || (v0+w0)<-2000)
	{ 
		device.SetCommand(_G, 1, 0); device.SetCommand(_G, 2, 0);
	}
	
	status = device.SetCommand(_G, 1, int(w0));	
	status = device.SetCommand(_G, 2, int(v0));
}

int main(int argc, char* argv[])
{	

	ros::init(argc, argv, "vehicle_command");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
		
	ROS_INFO("\n\n--- Roboteq Motor Controller Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	
	status = device.Connect("/dev/ttyACM0");

	while (status != RQ_SUCCESS && ros::ok())
	{
		ROS_INFO("Error connecting to device: %d\n", status);
		ROS_INFO("Attempting server restart...");
		usleep(1000000);
		device.Disconnect();

		status = device.Connect("/dev/ttyACM0");
		if (status == 0) {
			ROS_INFO("Connection re-established...");
		}
	}
	
	int mode=0;
	device.GetConfig(_MXMD, mode);
	ROS_INFO("Operating mode: %d", mode);
	ros::Duration(1.0).sleep();
	
	device.SetConfig(_MXRPM, 1, 2000);
	device.SetConfig(_MXRPM, 2, 2000);

	
	while(ros::ok()){
		
		motorcommand();

		loop_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
