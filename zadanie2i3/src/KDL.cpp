#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <signal.h>
#include <stdio.h>
#include <iostream>

double pozycjaZadana[3];

// Funkcja do pobrania wartości zadanych
void callback(const sensor_msgs::JointStateConstPtr &msg) 
{
	for (int i = 0; i < 3; i++)
	{
		pozycjaZadana[i]=msg->position[i];
	}
}

int main(int argc, char **argv)
{
	
	// Inicjalizacja ros-a
	ros::init(argc,argv,"KDL_DKIN");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/PoseStampedKDL", 1000);
	ros::Subscriber sub=nh.subscribe<sensor_msgs::JointState>("/joint_states", 1000, boost::bind(callback,_1));
	ros::Rate rate(10);

	double dlugosc;
	while(ros::ok())
	{
		ros::spinOnce(); // Pobranie informacji od Joint_State_Publishera
		
		
		// Pobranie danych z serwera parametrów

		//Tutaj musi być obliczanie parametrów do wysłania z użyciem KDL i z uwzględnieniem ograniczeń
		
		geometry_msgs::PoseStamped doWyslania;
		doWyslania.header.frame_id="link1";
		doWyslania.pose.position.x=pozycjaZadana[0];
		doWyslania.pose.position.y=pozycjaZadana[1];
		doWyslania.pose.position.z=pozycjaZadana[2];
		doWyslania.pose.orientation.w=pozycjaZadana[2];
		doWyslania.pose.orientation.x=pozycjaZadana[2];
		doWyslania.pose.orientation.y=pozycjaZadana[2];
		doWyslania.pose.orientation.z=pozycjaZadana[2];
		
		
		pub.publish(doWyslania);
		rate.sleep();
	}

	return 0;
}
