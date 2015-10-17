#include "ros/ros.h"
#include "roboteq/RoboteqDevice.h"
#include "vectornav/vn100_msg.h"
#include "roboteq/roboteq_msg.h"
#include "roboteq/ErrorCodes.h"
#include "roboteq/Constants.h"
#include "math.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

using namespace std;

float v0 = 0;
float desired_heading = 74.2;
float vel = 0.5;

const float RAD_TO_DEG = 180/3.141592;
const float DEG_TO_RAD = 3.141592/180;

const float Lr = 0.70;       
const float CIRC = 1.17; 
const float RED = 91.0;

const float correction_factor = 5.0/4.6;
const int turn_direction = -1; 
const int power_direction = -1; 

float w, v;

int status=0;
RoboteqDevice device;
 
const float PID_RESPONSE_SPEED = (4.0/12.0); 

const float P=2.0, I=1.0, D=0.1;
float error_p[2], error_d, error_i; 

float compute_error(float yaw){
	yaw -= desired_heading;

	if(yaw>180) yaw -= 360;
	else if (yaw<-180) yaw += 360;

	return yaw;
}

void motorcommand(){
	float pid = (PID_RESPONSE_SPEED*P*error_p[1] + I*error_i + D*error_d);
	//cout<<pid<<endl;
	int _max = 0; 
	device.GetConfig(_MXRPM, _max); 
	float max = _max;
	
	w = pid*DEG_TO_RAD*(Lr/2)*(60/CIRC)*(float)turn_direction*(1000/max)*RED; 
	
	v0 = (vel)*(1-fabs(pid)/180);
			
	v = v0*(60/CIRC)*(float)power_direction*(1000/max)*RED;                    		
			
	if( (((v+w)*power_direction)>2000) || (((v-w)*power_direction ) > 2000) ){ 
		device.SetCommand(_G, 1, 0); device.SetCommand(_G, 2, 0);
	}
	
	status = device.SetCommand(_G, 1, int(w));	
	status = device.SetCommand(_G, 2, int(v));
	
	v0 = 0;
}

void PID(const vectornav::vn100_msg::ConstPtr& info){	
	float yaw = info->linear.theta;
	
	float dt = 0.05;
	
	error_p[0] = error_p[1]; 
	error_p[1] = compute_error(yaw);
		
	error_d = (error_p[1]-error_p[0])/dt;
	if ((error_d > 30.0) || (error_d < (-30.0))) error_d = 0;
	
	error_i += (error_p[0] + error_p[1])*dt/2;	
	if ((error_i > 10.0) || (error_i < (-10.0))) error_i = 0;
	
	motorcommand();
}

void deshead(const std_msgs::Float32::ConstPtr& msg_in){
	desired_heading = msg_in->data;
	cout<<desired_heading<<endl;
}

void no_obs(const std_msgs::String::ConstPtr& msg_data){
	string safety_message = msg_data->data;
	//if(safety_message == "no near obstacle") vel=0.50;
	//vel = 0;
}

int main(int argc, char* argv[]){	
	
	ros::init(argc, argv, "trajec_design");
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
	
	geometry_msgs::Twist my_twist;
	
	device.SetConfig(_MXRPM, 1, 5000);
	device.SetConfig(_MXRPM, 2, 5000);
	
	ros::Subscriber heading = n.subscribe("/vn100_monitor", 1, PID);
	ros::Subscriber desired_heading = n.subscribe("/desired_bearing", 1, deshead);
	ros::Subscriber lessDist=n.subscribe("/message", 1, no_obs);
	
	ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/husky/cmd_vel",1);
	
	while(ros::ok()){
		ros::spinOnce();
		//motorcommand();
		my_twist.linear.x=v;
		my_twist.angular.z=w;
		pub.publish(my_twist);
		//ROS_INFO("%f", my_twist.linear.x);
		//ROS_INFO("%f", my_twist.angular.z);
		loop_rate.sleep();
	}
	
	return 0;
}
