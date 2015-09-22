#include "ros/ros.h"
#include "tf/tf.h"

#include <stdio.h>
#include <iostream>

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"


using namespace std;

nav_msgs::Path output;

void vizCallback(const nav_msgs::Odometry::ConstPtr& info)
{		
	static geometry_msgs::PoseStamped current;

	//Header Data
	current.header.seq = info->header.seq;
	current.header.stamp = info->header.stamp;

	//Pose Data
	current.pose = info->pose.pose;

	//Appending to Pose Array
	output.poses.push_back(current);
}

int main(int argc, char* argv[]){
	
	///ROS Node Initializations
	ros::init(argc, argv, "odom_viz");
	ros::NodeHandle n;
	ros::Publisher vizPublisher = n.advertise<nav_msgs::Path>("/gps_viz", 1);
	ros::Rate loop_rate(30);

	ros::Subscriber vizMonitor = n.subscribe("/gps", 5, vizCallback);
	
	///Publishing Loop
	output.header.frame_id = "odom_combined";
	
	while(ros::ok()){
		ros::spinOnce();

		output.header.stamp = ros::Time::now();
		output.header.seq++;

		/*Publishing Data*/
		vizPublisher.publish(output);
		
		/*Other Stuff*/
		loop_rate.sleep();
	}	

	return 0;
}
