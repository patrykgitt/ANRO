#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "zadanie4/oint_control_srv.h"
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

double const PI=M_PI;
double f=10;
double Tp=1/f;
double x,y,z,xo=0,yo=0,zo=0,wo=1;
double x0=0;
double yy0=0,z0=0,xo0=0,yo0=0,zo0=0,wo0=0;
double px,py,pz=0,ox=0,oy=0,oz,ow,czas;
bool czyInterpolujemy = true;
double fi,kx,ky,kz;
double ox1=0.0,oy1=0.0,oz1=0.0,ow1=1.0;
int n=0; // Liczba iteracji pętli
int k=0; // Indeks iteracji pętli

// Procedura do obsługi żądania
bool procedura(zadanie4::oint_control_srv::Request &req, zadanie4::oint_control_srv::Response &res)
{
	if(req.czasRuchu <= 0)
	{
		// Wysłanie odpowiedzi negatywnej do klienta
		res.rezultat="Podaj dodatni czas trwania ruchu.";
		ROS_WARN("Podano nieprawidlowy czas ruchu.");
		return true;
	}
	px=req.x;
	py=req.y;
	pz=req.z;
	ox=req.ox;
	oy=req.oy;
	oz=req.oz;
	ow=req.ow;
	czas=req.czasRuchu;
	czyInterpolujemy=true;	
	n=czas/Tp; 
	k=1;

	x0=x;
	yy0=y;
	z0=z;
	
	ox1=xo;
	oy1=yo;
	oz1=zo;
	ow1=wo;
	
	fi=2*acos(ow);
	double sinfipol=sin(fi/2);
	if(sinfipol=0)
	{
		kx=ox/sinfipol;
		ky=oy/sinfipol;
		kz=oz/sinfipol;
	}
	else kx=ky=kz=0;
	
	return true;
}



int main(int argc, char **argv)
{
	// Inicjalizacja ros-a
	ros::init(argc,argv,"oint");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("/PoseStamped", 1000);
	ros::ServiceServer serwer = nh.advertiseService("oint_control_srv",procedura);
	ros::Rate rate(f);
visualization_msgs::Marker marker;
marker.id=0;
marker.header.frame_id = "/base_link";
    marker.ns = "final_pos";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.5;
        marker.color.a = 1;

//marker.type=visualization_msgs::Marker::POINTS;
//visualization_msgs::MarkerArray markerArray;
ros::Publisher pub2=nh.advertise<visualization_msgs::Marker>("/visualization_marker", 100);
	while(ros::ok())
	{
		ros::spinOnce();
		if(!czyInterpolujemy) continue;

		if(n==0 && k==1)
		{
			x=px;
			y=py;
			z=pz;
			xo=ox;
			yo=oy;
			zo=oz;
			wo=ow;
		/*	ox1=ox;
			oy1=oy;
			oz1=oz;
			ow1=ow;*/
		}
		else if(k<1 || n < 1)
		{
			czyInterpolujemy=false;
		}

		else if (czyInterpolujemy)
		{
			x=x0+((px-x0)/czas)*k*Tp;
			y=yy0+((py-yy0)/czas)*k*Tp;
			z=z0+((pz-z0)/czas)*k*Tp;
			
			double ox2=kx*sin(k*fi/(2*n));
			double oy2=ky*sin(k*fi/(2*n));
			double oz2=kz*sin(k*fi/(2*n));
			double ow2=cos(k*fi/(2*n));
			
			xo=ow1*ox2+ox1*ow2+oy1*oz2-oz1*oy2;
			yo=ow1*oy2+oy1*ow2+oz1*ox2-ox1*oz2;
			zo=ow1*oz2+oz1*ow2+ox1*oy2-oy1*ox2;
			wo=ow1*ow2-ox1*ox2-oy1*oy2-oz1*oz2;			
		}

		k++;
		if (k>=n)
		{	
			x=px;
			y=py;
			z=pz;
			n=0;
			k=0;
			czyInterpolujemy=false;
			/*ox1=ox;
			oy1=oy;
			oz1=oz;
			ow1=ow;*/
		}

	    marker.id++;
	
		geometry_msgs::PoseStamped doWyslania;
		doWyslania.header.frame_id="/base_link";
		doWyslania.pose.position.x=x;
		doWyslania.pose.position.y=y;
		doWyslania.pose.position.z=z;
		doWyslania.pose.orientation.w=wo;
		doWyslania.pose.orientation.x=xo;
		doWyslania.pose.orientation.y=yo;
		doWyslania.pose.orientation.z=zo;
	marker.pose=doWyslania.pose;
/*geometry_msgs::Point punkt;
punkt.x=px;
punkt.y=py;
punkt.z=pz;
marker.points.push_back(punkt);	*/	
		pub2.publish(marker);
		pub.publish(doWyslania);
		rate.sleep();
	}

	return 0;
}
