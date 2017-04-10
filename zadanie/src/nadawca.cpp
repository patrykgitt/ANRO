#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <iostream>
int main(int argc, char ** argv)
{
	ros::init(argc,argv,"nadawca");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	ros::Rate rate(10);

	int kfd=0;
	struct termios cooked,raw;
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	raw.c_cc[VEOL]=1;
	raw.c_cc[VEOF]=2;
	tcsetattr(kfd, TCSANOW, &raw);	

	std::cout<<"Uzyj odpowiednich klawiszy, zeby poruszac sie w wybranym kierunku:"<< std::endl;
	std::cout << "a-lewo, s-tyl, d-prawo, w-przod" << std::endl;;
	while(ros::ok())
		{
			geometry_msgs::Twist msg;			
			char klawisz;

			if(read(kfd,&klawisz,1)>=0);
			{
			if(klawisz=='w') //góra
				{
					msg.linear.x=2.0;
				}
			if(klawisz=='s') //dół
				{
					msg.linear.x=-2.0;
				}
			if(klawisz=='a') //lewo
				{
					msg.angular.z=2.0;
				}
			if(klawisz=='d') //prawo
				{
					msg.angular.z=-2.0;
				}
			}
			pub.publish(msg);
			rate.sleep();			
		}
	return 0;
}
