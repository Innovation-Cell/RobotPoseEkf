#include "ros/ros.h"
#include "vectornav/utmcoord.h"
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <math.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

using namespace std;

char flag='a';double difference;
double northing=0,easting=0;
double init_northing=0,init_easting=0;
bool checking=0;

void utmcallback(const vectornav::utmcoord::ConstPtr &msg)
{
	northing=msg->northing;
	easting=msg->easting;
}


int main(int argc,char **argv)
{
	ros::init(argc,argv,"utmdiff");
	ros::NodeHandle n;
	ros::Subscriber utmvalue=n.subscribe("utm_coordinates",5,utmcallback);
	ros::Publisher utmdiff=n.advertise<std_msgs::Float64>("gps_displacement",1);
	ros::Rate loop_rate(20);
//	while(1)
//	{	
	//	cout<<"Press i to set initial position"<<endl;
		//cin>>flag;
		//if(flag=='i')
//		break;
	//	cout<<"Invalid input"<<endl;
	//}
	std_msgs::Float64 diff;
	while(ros::ok())
	{
		if(flag=='i'&&checking==0)
		{
			init_northing=northing;
			init_easting=easting;
			checking=1;
		}
		else if(flag!='i')
		{
			cout<<"Press i to set initial position"<<endl;
			cin>>flag;
		}
		difference=sqrt(pow((northing-init_northing),2.0)+pow((easting-init_easting),2.0));
		diff.data=difference;
		utmdiff.publish(diff);
		ros::spinOnce();
		loop_rate.sleep();
	}		
		
}
