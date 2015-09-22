#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

#include <stdio.h>
#include <iostream>

#include "nav_msgs/Odometry.h"


using namespace std;



int main(int argc, char* argv[]){

	///ROS Node Initializations
		/*Initializing Node*/
		ros::init(argc, argv, "Dummy");
		ros::NodeHandle n;
		
		/*Get Topic Name From Parameter Server*/
		string topicName;
		if(n.getParam("/dummy/topic", topicName)){
			ROS_INFO("Got param 'topic': %s", topicName.c_str());
		}
		else{
			ROS_ERROR("Failed to get param 'topic'");
		}
		
		/*Set Publisher With Topic Name*/
		ros::Publisher dummyOdomPublisher = n.advertise<nav_msgs::Odometry>(topicName, 1);
		ros::Rate loop_rate(20);
	
	///Publishing to /tf_static Topic
	static tf::TransformBroadcaster br;
	tf::Transform transform(tf::Quaternion(0.0, 0.0, 0.0, 1.0), tf::Vector3(0.1, 0.0, 0.0));
	
	///Fill in Dummy Values
		nav_msgs::Odometry output;
		
		/*Fixed Header Data*/
		output.header.frame_id = "odom_combined";
		output.child_frame_id = "base_footprint";
		
		/*Dummy Pose Data*/
		output.pose.pose.position.x=0; output.pose.pose.position.y=0; output.pose.pose.position.z=0;
		output.pose.pose.orientation.x=1; output.pose.pose.orientation.y=0; output.pose.pose.orientation.z=0; output.pose.pose.orientation.w=0;
		for(int i=0; i<36; i++){
			if((i/6)==(i%6)) output.pose.covariance[i]=9999;
			else output.pose.covariance[i]=0;
		}
		
		/*Dummy Twist Data*/
		output.twist.twist.angular.x=0; output.twist.twist.angular.y=0; output.twist.twist.angular.z=0;
		output.twist.twist.linear.x=0; output.twist.twist.linear.y=0; output.twist.twist.linear.z=0;
		for(int i=0; i<36; i++){
			if((i/6)==(i%6)) output.twist.covariance[i]=9999;
			else output.twist.covariance[i]=0;
		}
		


	///Publishing Loop
		while(ros::ok()) {
			ros::Time current_time;
			current_time = ros::Time::now();

			output.header.stamp = current_time;
			output.header.seq++;
		
			dummyOdomPublisher.publish(output);

			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_footprint", "/dummy"));

			
			loop_rate.sleep();
		}
	
	return 0;
}
