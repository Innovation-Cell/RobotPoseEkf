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
#include "std_msgs/Float32MultiArray.h"
#include "vector"
#include "videofeed/multi_lane.h"
#include "videofeed/lane.h"
#include <algorithm>

using namespace std;

//vector<vector<double> > lane_data;

float heading, head(0.0), slope, destination(0.0), ReqDistFromLane(0.10), road_width(3.0), dist_from_lane ;
int noOfLanes, noOfLaneSeg, leftLane;
bool flagIP(false);

float v0 = 0;
//float desired_heading = 0;
float desired_distance = 0;
float vel = 0.50;
//float yaw = 0;
//float slope_angle = 0;
//float minum_dist = 0;

const float pi = 3.141592;
const float pi_2 = 3.141592/2.0;
const float RAD_TO_DEG = 180/3.141592;
const float DEG_TO_RAD = 3.141592/180;

const float Lr = 0.62;       
const float CIRC = 3.141592*0.38; 
const float RED = 66.0;

const float correction_factor = 5.0/4.6;
const int turn_direction = -1; 
const int power_direction = 1; 

float w, v;

int status=0;

RoboteqDevice device;
 
const float PID_RESPONSE_SPEED = (4.0/12.0); 

const float P=2.0, I=0.1, D=0.1;
float error_p[2], error_d, error_i; 

void motorcommand(){
	float pid = (PID_RESPONSE_SPEED*P*error_p[1] + I*error_i + D*error_d);
	if (pid > 20 ) pid = 20.0;
	if (pid < -20) pid = -20.0;
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

void PID(){		
	float dt = 0.05;
	
	error_p[0] = error_p[1]; 
	error_p[1] = (heading - 90.0);
		
	error_d = (error_p[1]-error_p[0])/dt;
	if ((error_d > 30.0) || (error_d < (-30.0))) error_d = 0;
	
	error_i += (error_p[0] + error_p[1])*dt/2;	
	if ((error_i > 10.0) || (error_i < (-10.0))) error_i = 0;

	motorcommand();
}

void turnToDest()
{
  heading = destination;
  cout<<"turn your car at "<<heading<<" with respect to true north\n";
  PID();
}

void imucall(const std_msgs::Float32::ConstPtr &info)
{
  head=info->data;
  if(flagIP==false) turnToDest();
}

void lane_follow()
{
  heading = head+slope;
  cout<<"keep your car at "<<heading<<" with respect to true north\n";
  PID();
}

void dist_check()
{
  if (dist_from_lane<= ReqDistFromLane)
  {
    heading = head + slope + 5*leftLane;
    cout<<"keep your car at "<<heading<<" with respect to true north to avoid lane.\n";
    PID();
  }
  else if(dist_from_lane>= ReqDistFromLane)
  {
    heading = head + slope - 5*leftLane;
    cout<<"keep your car at "<<heading<<" with respect to true north to avoid lane.\n";
    PID();
  }
  else lane_follow();
}


void mask_callback(const videofeed::multi_lane::ConstPtr& msg){
	flagIP=true;
    noOfLanes=msg->num_of_lanes;
    noOfLaneSeg=msg->Lanes[0].number;
    dist_from_lane = msg->Lanes[0].min_dist;

   float point1Dist(msg->Lanes[0].dist[noOfLaneSeg-1]), point2Dist(msg->Lanes[0].dist[noOfLaneSeg-2]), dest_angle(destination-head);
   float point1Theta(msg->Lanes[0].theta[noOfLaneSeg-1]), point2Theta(msg->Lanes[0].theta[noOfLaneSeg-2]);

   leftLane=(point1Theta<0)? 1:-1;

   for (int i=(noOfLaneSeg-1); i>=0; i--)
   {
   	 float alpha(max(point1Theta,point2Theta)), beta(min(point1Theta, point2Theta));
     float r1((alpha==point1Theta)? point1Dist:point2Dist), r2((alpha==point1Theta)? point2Dist:point1Dist);
     float del_Y(r1*cos(alpha)-r2*cos(beta)),  del_X(r1*sin(alpha)-r2*sin(beta));
	 slope += del_X/(del_Y*(noOfLaneSeg-1));
   }

   slope = (180/M_PI)*atan(slope);

   cout<<slope<<"=slope of line\n";

   (fabsf(dest_angle-slope)<90)? dist_check(): turnToDest();
}

/*void current_head(const vectornav::vn100_msg::ConstPtr& info){	
	yaw = info->linear.theta;
}*/

void des_head(const std_msgs::Float32::ConstPtr& msg_in){
	destination = msg_in->data;
}

int main(int argc, char* argv[]){	
	
	ros::init(argc, argv, "lane_follower");
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
	
	device.SetConfig(_MXRPM, 1, 5000);
	device.SetConfig(_MXRPM, 2, 5000);

	ros::Subscriber heading = n.subscribe("/vn100_monitor", 1, imucall);
	ros::Subscriber desired_heading = n.subscribe("/desired_bearing", 1, des_head);
	ros::Subscriber lanedata = n.subscribe("/outdataIP", 1, mask_callback);
	
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
