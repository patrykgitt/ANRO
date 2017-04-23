#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <math.h>

double pozycjaZadana[3];
double const PI=M_PI;

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
	ros::init(argc,argv,"KDL_DKIN");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/PoseStampedKDL", 1000);
	ros::Subscriber sub=nh.subscribe<sensor_msgs::JointState>("/joint_states", 1000, boost::bind(callback,_1));
	ros::Rate rate(30);


	// Pobranie danych z serwera parametrów
	double a1=1.0,a2=3.0; // Rzeczywiste wartości do testów
	
	while(ros::ok())
	{
		ros::spinOnce(); // Pobranie informacji od Joint_State_Publisher
		
		double x,y,z=0; // Współrzędne końcówki
		
		KDL::Chain chain; // Stworzenie łańcucha kinematycznego

		double teta1=pozycjaZadana[0];
		double teta2 = pozycjaZadana[1];
		double d3 = pozycjaZadana[2];
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a1, 0, 0, teta1)));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a2, PI, 0, teta2)));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0, 0, d3, 0)));
		x = chain.getSegment(2).getFrameToTip().p.data[0];
		y = chain.getSegment(2).getFrameToTip().p.data[1]; 
		
		geometry_msgs::PoseStamped doWyslania;
		doWyslania.header.frame_id="arm3";
		doWyslania.pose.position.x=x;
		doWyslania.pose.position.y=y+2.0;
		doWyslania.pose.position.z=z;
		doWyslania.pose.orientation.w=0.707;
		doWyslania.pose.orientation.x=0.0;
		doWyslania.pose.orientation.y=0.0;
		doWyslania.pose.orientation.z=-0.707;
		
		
		pub.publish(doWyslania);
		rate.sleep();
	}

	return 0;
}
