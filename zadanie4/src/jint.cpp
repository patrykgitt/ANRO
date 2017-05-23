#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char **argv)
{
	// Inicjalizacja ros-a
	ros::init(argc,argv,"NONKDL_DKIN");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<sensor_msgs::JointState>("/Jint", 1000);
	//ros::ServiceServer serwer = nh.advertiseService(); // Rodzaj serwera
	ros::Rate rate(30);

	while(ros::ok())
	{
		
		





		sensor_msgs::JointState doWyslania;
		// Tworzenie wiadomości do wysłania		


		pub.publish(doWyslania);
		rate.sleep();
		
	}

	return 0;
}
