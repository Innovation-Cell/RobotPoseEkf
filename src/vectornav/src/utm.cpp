#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "gps_common/conversions.h"
#include <cstring>
#include "std_msgs/String.h"
#include <vectornav/utmcoord.h>

using namespace std;
double lat,lon,alt,error;
double UTMnorthing,UTMeasting;
string zone;

void lla_callback(const sensor_msgs::NavSatFix::ConstPtr& lla)
{
	lat=lla->latitude;
	lon=lla->longitude;
	alt=lla->altitude;
	error=lla->position_covariance[0];
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"lla_to_utm");
	ros::NodeHandle n;
	ros::Subscriber lla=n.subscribe("/vn200_insdata_lla",5,lla_callback);
	ros::Publisher utm=n.advertise<vectornav::utmcoord>("/utm_coordinates",1);
	ros::Rate loop_rate(20);
	vectornav::utmcoord coord;
	while(ros::ok())
	{
		gps_common::LLtoUTM(lat,lon,UTMnorthing,UTMeasting,zone);
		coord.header.stamp=ros::Time::now();
		coord.header.seq++;
		coord.header.frame_id="/odom";
		coord.northing=UTMnorthing;
		coord.easting=UTMeasting;
		coord.UTMzone.data=zone;
		coord.covariance[0]=coord.covariance[3]=error;
		coord.covariance[1]=coord.covariance[2]=0;
		//for(int i=0;i<5;i++)
		//	coord.UTMzone[i]=zone[i];
		utm.publish(coord);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
