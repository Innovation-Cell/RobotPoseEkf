#include "ros/ros.h"
#include "roboteq/RoboteqDevice.h"
#include "vectornav/vn100_msg.h"
#include "roboteq/roboteq_msg.h"
#include "roboteq/ErrorCodes.h"
#include "roboteq/Constants.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "vector"
#include "algorithm"

using namespace std;

float v0 = 0;
float vel = 0.1;
float yaw = 0;
float desired_heading = 0;
float w0 = 0;
int status = 0;

const float pi = 3.141592;
const float pi_2 = 3.141592/2.0;
const float RAD_TO_DEG = 180/3.141592;
const float DEG_TO_RAD = 3.141592/180;

const float Lr = 0.62;       
const float CIRC = 3.141592*0.38; 
const float RED = 66.0;

const float correction_factor = 5.0/4.6;
const int turn_direction = 1; 
const int power_direction = 1; 

float w, v;

RoboteqDevice device;

void motorcommand(){

	int _max = 0; 
	device.GetConfig(_MXRPM, _max); 
	float max = _max;

	w = w0*DEG_TO_RAD*(Lr/2)*(60/CIRC)*(float)turn_direction*(1000/max)*RED; 
			
	v = v0*(60/CIRC)*(float)power_direction*(1000/max)*RED;                    		
			
	if( (((v+w)*power_direction)>2000) || (((v-w)*power_direction ) > 2000) ){ 
		device.SetCommand(_G, 1, 0); device.SetCommand(_G, 2, 0);
	}
	
	device.SetCommand(_G, 1, int(w));	
	device.SetCommand(_G, 2, int(v));
	
	v0 = 0;
	w0 = 0;
}

void lidar_data(const std_msgs::Float32MultiArray::ConstPtr &msg){
	int clusters = msg->data[0];
	cout<<clusters<<endl;
	double cluster_data[clusters][4];

	for (int i=0; i<clusters; i++){
		cluster_data[i][0] = msg->data[6*i+1];		
		cluster_data[i][1] = msg->data[6*i+2];
		cluster_data[i][2] = msg->data[6*i+3];		
		cluster_data[i][3] = msg->data[6*i+4];
	} 
}

void current_head(const vectornav::vn100_msg::ConstPtr& info){	
	yaw = info->linear.theta;
}

void des_head(const std_msgs::Float32::ConstPtr& msg_in){
	desired_heading = msg_in->data;
}

int main(int argc, char* argv[]){	
	
	ros::init(argc, argv, "obs_avoid");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
		
	ROS_INFO("\n\n--- Roboteq Motor Controller Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	
	status = device.Connect("/dev/ttyACM2");

	while (status != RQ_SUCCESS && ros::ok())
	{
		ROS_INFO("Error connecting to device: %d\n", status);
		ROS_INFO("Attempting server restart...");
		usleep(1000000);
		device.Disconnect();

		status = device.Connect("/dev/ttyACM2");
		if (status == 0) {
			ROS_INFO("Connection re-established...");
		}
	}
	
	int mode=0;
	device.GetConfig(_MXMD, mode);
	ROS_INFO("Operating mode: %d", mode);
	ros::Duration(1.0).sleep();
	
	device.SetConfig(_MXRPM, 1, 5000);
	device.SetConfig(_MXRPM, 2, 5000);

	ros::Subscriber heading = n.subscribe("/vn100_monitor", 1, current_head);
	ros::Subscriber desired_heading = n.subscribe("/desired_bearing", 1, des_head);
	ros::Subscriber lidardata = n.subscribe("/cluster_info_pub", 1, lidar_data);
	
	ros::Publisher  pub = n.advertise<geometry_msgs::Twist>("/husky/cmd_vel",1);
	
	geometry_msgs::Twist my_twist;

	while(ros::ok()){
		my_twist.linear.x=v;
		my_twist.angular.z=w;
		pub.publish(my_twist);
		//ROS_INFO("%f", my_twist.linear.x);
		//ROS_INFO("%f", my_twist.angular.z);
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}