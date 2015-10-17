#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geodesy/utm.h"

#include <stdio.h>
#include <iostream>

#include "vectornav.h"
#include "vectornav/vn200_msg.h"


using namespace std;
vectornav::vn200_msg output;


const int SERIAL_BAUD_RATE = 115200;
const char* COM_PORT = "/dev/ttyUSB0";

void int_to_binary(int integer){
	for(int i=0; i<7; i++){
		output.status[i] = integer%2;
		integer /= 2;
	}
}

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
	ros::Publisher latlonger = n.advertise<vectornav::vn200_msg>("/vn200_monitor", 1);
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
	

	
	
	///Debugging Data Output Type
	unsigned int type;
	vn200_getAsynchronousDataOutputType(&vn200, &type);
	ROS_INFO("Asynchronous Data Output Type Set to: %d\n\n", type);
	sleep(1);
	
	
	
	///Publishing Loop
	ros::Time current_time; 
	
	output.NED.header.frame_id="ypr";
		
	while(ros::ok()){
		
		/*Getting Data*/
		if(vn200.isConnected){
			vn200_getInsSolution(&vn200, &gpsTime, &gpsWeek, &status, &ypr, &latitudeLongitudeAltitude, &nedVelocity, &attitudeUncertainty, &positionUncertainty, &velocityUncertainty);
			current_time = ros::Time::now();
			//ROS_INFO("Yaw: %f", ypr.c0); 
			//ROS_INFO("Latitude: %f", latitudeLongitudeAltitude.c0);
			//ROS_INFO("North Velocity: %f\n", nedVelocity.c0);
		}
		else ROS_INFO("Connection Error\n----------------------------\n");
				
		/*Filling in Data*/
			//Header Info//
			output.NED.header.stamp=current_time; output.NED.header.seq++;
			//Data Values//
			output.LLA.latitude=latitudeLongitudeAltitude.c0; output.LLA.longitude=latitudeLongitudeAltitude.c1; output.LLA.altitude=latitudeLongitudeAltitude.c2;
			output.NED.vector.x=nedVelocity.c0; output.NED.vector.y=nedVelocity.c1; output.NED.vector.z=nedVelocity.c2;
			int_to_binary(status); 
			
			for(int i=6; i>=0; i--){
				printf("%d", output.status[i]);
			}
			printf("\n");
		/*Publishing Data*/
		latlonger.publish(output);
		
		/*Other Stuff*/
		loop_rate.sleep();
	}	

	return 0;
}
