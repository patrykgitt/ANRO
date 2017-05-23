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
bool joint_state_aktywny=false;

// Funkcja do pobrania wartości zadanych
void callback(const sensor_msgs::JointStateConstPtr &msg) 
{
	joint_state_aktywny=true;
	for (int i = 0; i < 3; i++)
	{
		pozycjaZadana[i]=msg->position[i];
	}
	pozycjaZadana[0]+=PI/5; // Warunek początkowy
}

int main(int argc, char **argv)
{
	
	// Inicjalizacja ros-a
	ros::init(argc,argv,"KDL_DKIN");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/PoseStampedKDL", 1000);
	ros::Subscriber sub=nh.subscribe<sensor_msgs::JointState>("/joint_states", 1000, boost::bind(callback,_1));
	ros::Rate rate(30);


	double a1=1.0,a2=3.0; // Rzeczywiste wartości do testów
	double joint2_lower=-2.9,joint2_upper=2.9,joint3_lower=0.3,joint3_upper=3.0; // Dane domyślne 
	
	// Pobranie danych z serwera parametrów

	if(!nh.getParam("/arm1",a1))
		{
			ROS_WARN("Nie udalo sie pobrac parametrow. Uzyte zostana domyslne wartosci.");	
		}
	if(!nh.getParam("/arm2",a2))
		{
			ROS_WARN("Nie udalo sie pobrac parametrow. Uzyte zostana domyslne.wartosci");	
		}
	if(!nh.getParam("/joint2_lower",joint2_lower))
		{
			ROS_WARN("Nie udalo sie pobrac parametrow. Uzyte zostana domyslne wartosci.");	
		}
	if(!nh.getParam("/joint2_upper",joint2_upper))
		{
					ROS_WARN("Nie udalo sie pobrac parametrow. Uzyte zostana domyslne wartosci.");	
		}
	if(!nh.getParam("/joint3_lower",joint3_lower))
		{
					ROS_WARN("Nie udalo sie pobrac parametrow. Uzyte zostana domyslne wartosci.");
		}
	if(!nh.getParam("/joint3_upper",joint3_upper))
		{
					ROS_WARN("Nie udalo sie pobrac parametrow. Uzyte zostana domyslne wartosci.");
		}

	while(ros::ok())
	{
		ros::spinOnce(); // Pobranie informacji od Joint_State_Publishera
		if(!joint_state_aktywny) // Sprawdzamy, czy już odebraliśmy jakieś dane od joint_state_publishera
			continue;
		// Sprawdzanie, czy spełnione są ograniczenia kinematyczne manipulatora

		if(pozycjaZadana[1] > joint2_upper || pozycjaZadana[1] < joint2_lower)
		{
			ROS_ERROR("Wyznaczenie polozenia nie jest mozliwe.");
			continue;
		}
		
		if(pozycjaZadana[2] > joint3_upper || pozycjaZadana[2] < joint3_lower)
		{
			ROS_ERROR("Wyznaczenie polozenia nie jest mozliwe.");
			continue;
		}

		double x,y,z=0; // Współrzędne końcówki
		KDL::Chain chain; // Stworzenie łańcucha kinematycznego

		double teta1=pozycjaZadana[0];
		double teta2 = pozycjaZadana[1];
		double d3 = pozycjaZadana[2];

		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a1, 0, 0, teta1)));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(a2, PI, 0, teta2)));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ),KDL::Frame::DH(0, 0, d3, 0)));
		x = chain.getSegment(2).getFrameToTip().p.data[0];
		y = chain.getSegment(2).getFrameToTip().p.data[1]; 
		z = chain.getSegment(2).getFrameToTip().p.data[2];

		geometry_msgs::PoseStamped doWyslania;
		doWyslania.header.frame_id="arm3";
		doWyslania.pose.position.x=x;
		doWyslania.pose.position.y=y+2.0;
		doWyslania.pose.position.z=0;
		doWyslania.pose.orientation.w=0.707;
		doWyslania.pose.orientation.x=0.0;
		doWyslania.pose.orientation.y=0.0;
		doWyslania.pose.orientation.z=-0.707;
		
		
		pub.publish(doWyslania);
		rate.sleep();
	}

	return 0;
}
