#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>

double pozycjaZadana[3];

void callback(const sensor_msgs::JointStateConstPtr &msg)
{
	pozycjaZadana[0]=msg->position[1];
	std::cout<<pozycjaZadana[0];
}

int main(int argc, char **argv)
{
	std::cout<<"Jestem przed ";
	// Inicjalizacja ros-a
	ros::init(argc,argv,"NONKDL_DKIN");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/PoseStamped", 1000);
	std::cout<<"Jestem przed ";
	ros::Subscriber sub=nh.subscribe<sensor_msgs::JointState>("/joint_state_publisher", 1000, boost::bind(callback,_1));
	ros::Rate rate(10);

	while(ros::ok())
	{
		geometry_msgs::PoseStamped doWyslania;
		sensor_msgs::JointState odebrane;
		ros::spin();
	}

	// Deklaracje zmiennych
	
	



	ros::spin();
}
