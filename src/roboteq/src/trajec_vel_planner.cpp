#include "ros/ros.h"
#include "roboteq/RoboteqDevice.h"
#include "roboteq/roboteq_msg.h"
#include "roboteq/ErrorCodes.h"
#include "roboteq/Constants.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "vectornav/vn100_msg.h"

#include "cmath"
#include "string"

using namespace std;

std::vector<std::vector<float> > path_data;

int width(400); 
int height(800);
float *map_data = NULL;

int PATH_POINT_GAP(12), status(0);

float  map_res(0.05);
float MAX_VEL(0.5);
float velocity(0);
float err_old(0), err_new(0), errd(0), erri(0);
float curr_head(0);
float curr_xpos(0);
float curr_ypos(0);
float ang_vel(0);

string PORT = "/dev/ttyACM0";

const float RAD_TO_DEG(180/3.141592);
const float DEG_TO_RAD(3.141592/180);

const float Lr(0.70);       
const float CIRC(1.17); 
const float RED(91.0);

const float correction_factor(5.0/4.6);
const int direction_factor(-1);  
 
const float PID_RESPONSE_SPEED(4.0/12.0); 
const float P(2.0), I(1.0), D(0.1);

bool odom_received(false), currenthead_received(false), map_received(false);
bool newpath_required(true); 

RoboteqDevice device;

float yaw_err(float yaw_f, float yaw_i){
	yaw_f -= yaw_i;
	if(yaw_f>180) yaw_f -= 360;
	else if (yaw_f<-180) yaw_f += 360;
	return yaw_f;
}

void robot_cmd(float pid, float vel){
	int _max(0); 
	device.GetConfig(_MXRPM, _max); 
	float max = _max;
	
	ang_vel = pid*DEG_TO_RAD*(Lr/2)*(60/CIRC)*(float)direction_factor*(1000/max)*RED; 	
	vel = vel*(60/CIRC)*(float)direction_factor*(1000/max)*RED;                    		
			
	if( (((vel + ang_vel)*direction_factor)>2000) || (((vel - ang_vel)*direction_factor) > 2000) ){ 
		device.SetCommand(_G, 1, 0); device.SetCommand(_G, 2, 0);
	}
	
	device.SetCommand(_G, 1, int(ang_vel));	
	device.SetCommand(_G, 2, int(vel));
}

float PID(float yaw_e){	
	float dt(0.05), pid(0);
	
	err_old = err_new; 
	err_new = yaw_e;
		
	errd = (err_new-err_old)/dt;
	if ((errd>30.0) || (errd<(-30.0))) errd = 0;
	erri += (err_new + err_old)*dt/2;	
	if ((erri>10.0) || (erri<(-10.0))) erri = 0;

	pid = (PID_RESPONSE_SPEED*P*err_new + I*erri + D*errd);
	return pid;
}

bool check_occupancy_nextpos(float next_xpos, float next_ypos){
	bool check(false); 
	float dn(0), de(0);
	int xpos(0), ypos(0);
	dn = next_ypos - curr_ypos;
	de = next_xpos - curr_xpos;
	xpos = int((dn*cos(curr_head*DEG_TO_RAD) + de*sin(curr_head*DEG_TO_RAD))/map_res);
	ypos = (width/2-1) + int((dn*sin(curr_head*DEG_TO_RAD) - de*cos(curr_head*DEG_TO_RAD))/map_res);
	if (map_data[xpos*width + ypos] != 1) check = true; 
	return check;
}

void mapdata(const nav_msgs::OccupancyGrid::ConstPtr& map_in){
	width    = map_in->info.width;
	height   = map_in->info.height;
	map_res  = map_in->info.resolution;
	float* mdata = new float[width*height];
	for (int i=0;i<(width*height);i++){
		mdata[i] = map_in->data[i];
	}
	map_data = mdata;
	map_received = true;
}

void currenthead(const vectornav::vn100_msg::ConstPtr& theta_in){
	curr_head = theta_in->linear.theta;
	currenthead_received = true;
}

void odom_combined(const nav_msgs::Odometry::ConstPtr& odom_in){
	curr_ypos = odom_in->pose.pose.position.x;
	curr_xpos = odom_in->pose.pose.position.y;
	odom_received = true;
}

void get_path(const std_msgs::Float32MultiArray::ConstPtr& path_in){
	int pathsize(0);
	pathsize = path_in->data.size();
	
	path_data.resize(pathsize/2);
	for (int i=0;i<pathsize/2;i++){
		path_data[i].resize(2);
		path_data[i][0] = path_in->data[i];
		path_data[i][1]	= path_in->data[i+int(pathsize/2)];		
	}
	newpath_required = false;
}

int main(int argc, char* argv[]){	
	
	ros::init(argc, argv, "trajec_design");
	ros::NodeHandle n;

	ros::Rate loop_rate(10);
		
	ROS_INFO("\n\n--- Roboteq Motor Controller Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	
	status = device.Connect(PORT);

	while (status != RQ_SUCCESS && ros::ok())
	{
		ROS_INFO("Error connecting to device: %d\n", status);
		ROS_INFO("Attempting server restart...");
		usleep(1000000);
		device.Disconnect();

		status = device.Connect(PORT);
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

	geometry_msgs::Twist my_twist;
	std_msgs::Bool new_path;

	ros::Subscriber plannedpath = n.subscribe("/planned_path", 10, get_path);
	ros::Subscriber pose        = n.subscribe("/robot_pose_ekf/odom_combined", 10, odom_combined);
	ros::Subscriber sub         = n.subscribe("/localmap", 10, mapdata);

	ros::Publisher pub1 = n.advertise<geometry_msgs::Twist>("/husky/cmd_vel",1);
	ros::Publisher pub2 = n.advertise<std_msgs::Bool>("/newpath_required",1);

	float next_xpos(0), next_ypos(0), path_point_num(10);
	float dx(0), dy(0), dr(0), dt(0), yaw_f(0), dyaw(0), pid(0);
	bool check_next(false); 
	
	while(ros::ok()){
		if (odom_received == true && newpath_required == false && currenthead_received == true){
			if (fabs(dx) < .25 && fabs(dy) < .25){
				if (path_point_num <= path_data.size()){
					next_xpos = path_data[path_point_num][0];
					next_ypos = path_data[path_point_num][1];
					check_next = check_occupancy_nextpos(next_xpos, next_ypos);	
				}
				else {
					newpath_required = true;
				}
			}

			if (newpath_required == true || check_next == false) {
				path_point_num = 10;
				dx = 0; dy = 0;
				pid = 0; velocity = 0;
				newpath_required = true;
			}
			else {
				path_point_num += PATH_POINT_GAP;
				dx = next_xpos - curr_xpos;
				dy = next_ypos - curr_ypos;
				velocity = MAX_VEL;
				dr = pow((pow(dx,2)+pow(dy,2)),0.5);
				dt = dr/MAX_VEL;
				yaw_f = atan2(dy , dx)*RAD_TO_DEG;
				dyaw = yaw_err(yaw_f, curr_head);
				pid = PID(dyaw);
				robot_cmd(pid, velocity);
			}
		}

		my_twist.linear.x = MAX_VEL;
		my_twist.angular.z = ang_vel;

		new_path.data = newpath_required;

		pub1.publish(my_twist);
		pub2.publish(new_path);

		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
