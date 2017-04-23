#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <math.h>


double pozycjaZadana[3];
double const PI = M_PI;

// Funkcja do pobrania wartości zadanych
void callback(const sensor_msgs::JointStateConstPtr &msg) 
{
	for (int i = 0; i < 3; i++)
	{
		pozycjaZadana[i]=msg->position[i];
	}
	pozycjaZadana[0]+=PI/5;
}

int main(int argc, char **argv)
{
	
	// Inicjalizacja ros-a
	ros::init(argc,argv,"NONKDL_DKIN");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/PoseStampedNONKDL", 1000);
	ros::Subscriber sub=nh.subscribe<sensor_msgs::JointState>("/joint_states", 1000, boost::bind(callback,_1));
	ros::Rate rate(30);


	
	double a1=1.0,a2=3.0; // Wartości do pobrania z serwera parametrów
	// Pobranie danych z serwera parametrów
	
	
	
	while(ros::ok())
	{
		ros::spinOnce(); // Pobranie informacji od Joint_State_Publishera
		
		
		
		// Obliczanie parametrów do wysłania
		double x = a2*cos(pozycjaZadana[0]-pozycjaZadana[1])+a1*cos(pozycjaZadana[0]);
		double y = a2*sin(pozycjaZadana[0]-pozycjaZadana[1])+a1*sin(pozycjaZadana[0]);
		double z = -pozycjaZadana[2];
		// Ewentualnie dodać ograniczenia i logowanie


		
		geometry_msgs::PoseStamped doWyslania;
		doWyslania.header.frame_id="base_link";
		doWyslania.pose.position.x=x+2.0; // Długość strzałki to 2, chcemy, żeby grot strzałki pokrywał się z pozycją końcówki
		doWyslania.pose.position.y=y;
		doWyslania.pose.position.z=z;
		doWyslania.pose.orientation.w=0;
		doWyslania.pose.orientation.x=0; 
		doWyslania.pose.orientation.y=0;
		doWyslania.pose.orientation.z=1.0;
		
		
		pub.publish(doWyslania);
		rate.sleep();
	}

	return 0;
}
