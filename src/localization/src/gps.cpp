#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geodesy/utm.h"

#include <stdio.h>
#include <iostream>

#include "vectornav.h"

#include "nav_msgs/Odometry.h"


using namespace std;


double initialNorthing, initialEasting;

const int SERIAL_BAUD_RATE = 115200;
const char* COM_PORT = "/dev/ttyUSB1";


int main(int argc, char* argv[]){

	///Variable Declerations
	Vn200 vn200;
	Vn200CompositeData composedData;
	double gpsTime;
	unsigned short gpsWeek, status;
	VnVector3 ypr, latitudeLongitudeAltitude, nedVelocity;
	float attitudeUncertainty, positionUncertainty, velocityUncertainty;

	
	///ROS Node Initializations
	ros::init(argc, argv, "VN200_Interface");
	ros::NodeHandle n;
	ros::Publisher gpsOdomPublisher = n.advertise<nav_msgs::Odometry>("/gps", 1);
	ros::Rate loop_rate(20);

	
	///Connection to Vectornav
	int connect;
	ROS_INFO("Attempting Connection to IMU");
	connect = vn200_connect(&vn200, COM_PORT, SERIAL_BAUD_RATE);
	while(connect!=0){
		ROS_INFO("Error encountered:  %d\n Attempting again.", connect);
		usleep(1000);
		connect = vn200_connect(&vn200, COM_PORT, SERIAL_BAUD_RATE);
	}
	if(vn200.isConnected) ROS_INFO("Successfully connected!\n\n");
	
		
	///Publishing Loop
	ros::Time current_time; 
	
	geographic_msgs::GeoPoint forUTMConversion;
	geodesy::UTMPoint UTMConverted;

	nav_msgs::Odometry output;
	output.header.frame_id = "odom_combined";
	output.child_frame_id = "gps";
	output.pose.pose.orientation.x = 1; output.pose.pose.orientation.y = 0; output.pose.pose.orientation.z = 0;output.pose.pose.orientation.w = 0;
	for(int i=0; i<36; i++){
		if((i/6)==(i%6)) output.pose.covariance[i]=99999;
		else output.pose.covariance[i]=0;
	}
	output.twist.twist.angular.x=0; output.twist.twist.angular.y=0; output.twist.twist.angular.z=0;
	for(int i=0; i<36; i++){
		if((i/6)==(i%6)) output.twist.covariance[i]=99999;
		else output.twist.covariance[i]=0;
	}
	/*Determining Starting Point*/
	int i=0;
	while(i<50){
		if(vn200.isConnected){
			vn200_getInsSolution(&vn200, &gpsTime, &gpsWeek, &status, &ypr, &latitudeLongitudeAltitude, &nedVelocity, &attitudeUncertainty, &positionUncertainty, &velocityUncertainty);
			current_time = ros::Time::now();
		}
		else ROS_INFO("Connection Error\n----------------------------\n");
	
		if(latitudeLongitudeAltitude.c0!=0) {
			forUTMConversion.latitude = latitudeLongitudeAltitude.c0; forUTMConversion.longitude = latitudeLongitudeAltitude.c1; forUTMConversion.altitude = nanf("");
			fromMsg(forUTMConversion, UTMConverted);
		
			initialNorthing += UTMConverted.northing;
			initialEasting += UTMConverted.easting;
			
			i++;
		}
	}
	
	initialNorthing /= 50;
	initialEasting /= 50;
	
	/*Actual Loop*/
	while(ros::ok()){	
		/*Getting Data*/
		if(vn200.isConnected){
			vn200_getInsSolution(&vn200, &gpsTime, &gpsWeek, &status, &ypr, &latitudeLongitudeAltitude, &nedVelocity, &attitudeUncertainty, &positionUncertainty, &velocityUncertainty);
			current_time = ros::Time::now();
		}
		else ROS_INFO("Connection Error\n----------------------------\n");
				
		/*Filling in Data*/
			//Header Info//
			output.header.stamp=current_time; 
			output.header.seq++;
			//Pose Data//
				//Converting Lat-Long to X-Y
				forUTMConversion.latitude = latitudeLongitudeAltitude.c0; forUTMConversion.longitude = latitudeLongitudeAltitude.c1; forUTMConversion.altitude = latitudeLongitudeAltitude.c2;
				fromMsg(forUTMConversion, UTMConverted);
				//Into Message Variables
				output.pose.pose.position.x = UTMConverted.northing - initialNorthing;
				output.pose.pose.position.y = -(UTMConverted.easting - initialEasting);
				output.pose.pose.position.z = UTMConverted.altitude;              
				output.pose.covariance[0] = output.pose.covariance[7] = output.pose.covariance[14] = (double)positionUncertainty;  
									     
			//Twist Data//
			output.twist.twist.linear.x=nedVelocity.c0; 
			output.twist.twist.linear.y=nedVelocity.c1; 
			output.twist.twist.linear.z=-nedVelocity.c2;
			output.twist.covariance[0] = output.twist.covariance[7] = output.twist.covariance[14] = (double)velocityUncertainty;
		
		/*Publishing Data*/
		gpsOdomPublisher.publish(output);
		
		/*Other Stuff*/
		loop_rate.sleep();
	}	

	return 0;
}
