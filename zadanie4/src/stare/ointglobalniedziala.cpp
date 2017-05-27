#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "zadanie4/oint_control_srv.h"
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>


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
double alfa=0.0;
double beta=0.0;
double delta=0.0;

int n=0; // Liczba iteracji pętli
int k=0; // Indeks iteracji pętli

double t0,t1,t2,t3,t4,t5,xKat,yKat,zKat;
bool pierwszeUruchomienie=true;

class Macierz {
	public:
		double e[3][3];
		Macierz() {}
		Macierz(double kat, char os) {
			if(os=='x')
			{
				e[0][0]=1;
				e[0][1]=e[0][2]=e[1][0]=e[2][0]=0;
				e[1][1]=e[2][2]=cos(kat);
				e[2][1]=sin(kat);
				e[1][2]=-sin(kat);
			}
			if(os=='y')
			{
				e[1][1]=1;
				e[0][1]=e[2][1]=e[1][2]=e[1][0]=0;
				e[0][0]=e[2][2]=cos(kat);
				e[0][2]=sin(kat);
				e[2][0]=-sin(kat);
			}
			if(os=='z')
			{
				e[2][2]=1;
				e[0][2]=e[1][2]=e[2][0]=e[2][1]=0;
				e[1][1]=e[0][0]=cos(kat);
				e[0][1]=-sin(kat);
				e[1][0]=sin(kat);
			}
		}
		Macierz operator*(Macierz b) {
			Macierz wynik;
			wynik.e[0][0]=this->e[0][0]*b.e[0][0]+this->e[0][1]*b.e[1][0]+this->e[0][2]*b.e[2][0];
			wynik.e[0][1]=this->e[0][0]*b.e[0][1]+this->e[0][1]*b.e[1][1]+this->e[0][2]*b.e[2][1];
			wynik.e[0][2]=this->e[0][0]*b.e[0][2]+this->e[0][1]*b.e[1][2]+this->e[0][2]*b.e[2][2];
			wynik.e[1][0]=this->e[1][0]*b.e[0][0]+this->e[1][1]*b.e[1][0]+this->e[1][2]*b.e[2][0];
			wynik.e[1][1]=this->e[1][0]*b.e[0][1]+this->e[1][1]*b.e[1][1]+this->e[1][2]*b.e[2][1];
			wynik.e[1][2]=this->e[1][0]*b.e[0][2]+this->e[1][1]*b.e[1][2]+this->e[1][2]*b.e[2][2];
			wynik.e[2][0]=this->e[2][0]*b.e[0][0]+this->e[2][1]*b.e[1][0]+this->e[2][2]*b.e[2][0];
			wynik.e[2][1]=this->e[2][0]*b.e[0][1]+this->e[2][1]*b.e[1][1]+this->e[2][2]*b.e[2][1];
			wynik.e[2][2]=this->e[2][0]*b.e[0][2]+this->e[2][1]*b.e[1][2]+this->e[2][2]*b.e[2][2];
			return wynik;
		}
		Macierz T(Macierz a) {
			Macierz wynik;
			wynik.e[0][0]=a.e[0][0];
			wynik.e[0][1]=a.e[1][0];
			wynik.e[0][2]=a.e[2][0];
			wynik.e[1][0]=a.e[0][1];
			wynik.e[1][1]=a.e[1][1];
			wynik.e[1][2]=a.e[2][1];
			wynik.e[2][0]=a.e[0][2];
			wynik.e[2][1]=a.e[1][2];
			wynik.e[2][2]=a.e[2][2];
			return wynik;
		}
		void wyswietlMacierz()	{
			std::cout << " _    _ " << std::endl;
			std::cout << "|" << this->e[0][0] << " " << this->e[0][1] << " " << this->e[0][2] << "|" << std::endl;
			std::cout << "|" << this->e[1][0] << " " << this->e[1][1] << " " << this->e[1][2] << "|" << std::endl;
			std::cout << "|" << this->e[2][0] << " " << this->e[2][1] << " " << this->e[2][2] << "|" << std::endl;
			std::cout << " _   _ " << std::endl <<std::endl;
		}
};

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
	
	
	xKat=req.xkat;
	yKat=req.ykat;
	zKat=req.zkat;
	
	/*t0 = cos((zKat) * 0.5);
	t1 = sin((zKat) * 0.5);
	t2 = cos((xKat) * 0.5);
	t3 = sin((xKat) * 0.5);
	t4 = cos((yKat) * 0.5);
	t5 = sin((yKat) * 0.5);

	alfa=xKat;
	beta=yKat;
	delta=zKat;

	ow = t0 * t2 * t4 + t1 * t3 * t5;
	ox = t0 * t3 * t4 - t1 * t2 * t5;
	oy = t0 * t2 * t5 + t1 * t3 * t4;
	oz = t1 * t2 * t4 - t0 * t3 * t5;
*/
	
	
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
	
	/*fi=2*acos(ow);
	double sinfipol=sin(fi/2);
	if(sinfipol!=0)
	{
		kx=ox/sinfipol;
		ky=oy/sinfipol;
		kz=oz/sinfipol;
	}
	else kx=ky=kz=0;
	*/
	/*double r11 = cos(yKat-beta)*cos(zKat-delta);
	double r22=cos(zKat-delta)*cos(xKat-alfa)+sin(xKat-alfa)*sin(yKat-beta)*sin(zKat-delta);
	double r33=cos(xKat-alfa)*cos(yKat-beta);
	fi=acos((r11+r22+r33-1)/2);
	double r32=sin(xKat-alfa)*cos(yKat-beta);
	double r23=-cos(zKat-delta)*sin(xKat-alfa)+cos(xKat-alfa)*sin(yKat-beta)*sin(zKat-delta);
	double r13=sin(zKat-delta)*sin(xKat-alfa)+cos(xKat-alfa)*sin(yKat-beta)*cos(zKat-delta);
	double r31=-sin(yKat-beta);
	double r21=cos(yKat-beta)*sin(zKat-delta);
	double r12=-sin(zKat-delta)*cos(xKat-alfa)+sin(xKat-alfa)*sin(yKat-beta)*cos(zKat-delta);
	double sinfipol=sin(fi/2);
	*/
	
	Macierz Rz=Macierz(zKat,'z');
	Macierz Ry=Macierz(yKat,'y');

	Macierz Rx=Macierz(xKat,'x');

	Macierz Rdelta=Macierz(delta,'z');
	Macierz Rbeta=Macierz(beta,'y');
	Macierz Ralfa=Macierz(alfa,'x');
	Macierz R02=Rz*Ry;
	R02=R02*Rx;
	Macierz R01=Rdelta*Rbeta;
	R01=R01*Ralfa;
	Macierz R=R02*R01.T(R01);

R=Macierz(zKat-delta,'z')*Macierz(yKat-beta,'y')*Macierz(xKat-alfa,'x');


	fi=acos((R.e[0][0]+R.e[1][1]+R.e[2][2]-1)/2);
	std::cout <<"R01:" << std::endl;
	R01.wyswietlMacierz();
	std::cout <<"R02:" << std::endl;
R02=R*R01;
	R02.wyswietlMacierz();
	std::cout <<"R:" << std::endl;
	R.wyswietlMacierz();	
	std::cout << "fi=" << fi << std::endl;
	if(sin(fi)!=0)
	{
		 double kx1=(R.e[2][1]-R.e[1][2])/(2*sin(fi));
		double ky1=(R.e[0][2]-R.e[2][0])/(2*sin(fi));
		 double kz1=(R.e[1][0]-R.e[0][1])/(2*sin(fi));
		std::cout << "1k=[" <<kx1 << " " << ky1 << " " << kz1 <<"]" << std::endl;
		kx=kx1*cos(beta)*cos(delta)+ky1*(-sin(delta)*cos(alfa)+sin(alfa)*sin(beta)*cos(delta))+kz1*(sin(delta)*sin(alfa)+cos(alfa)*sin(beta)*cos(delta));
		ky=kx1*(cos(beta)*sin(delta))+ky1*(cos(delta)*cos(alfa)+sin(alfa)*sin(beta)*sin(delta))+kz1*(-cos(delta)*sin(alfa)+cos(alfa)*sin(beta)*sin(delta));
		kz=kx1*-sin(beta)+ky1*sin(alfa)*cos(beta)+kz1*cos(alfa)*cos(beta);
std::cout << "k=[" <<kx << " " << ky << " " << kz <<"]" << std::endl;
	}
	else kx=ky=kz=0;
	
	alfa=xKat;
	beta=yKat;
	delta=zKat;

	pierwszeUruchomienie=false;
	
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
	geometry_msgs::PoseStamped doWyslania;
	doWyslania.header.frame_id="/base_link";
	
//marker.type=visualization_msgs::Marker::POINTS;
//visualization_msgs::MarkerArray markerArray;
ros::Publisher pub2=nh.advertise<visualization_msgs::Marker>("/visualization_marker", 100);
	while(ros::ok())
	{
		ros::spinOnce();
		if(pierwszeUruchomienie == true){
			doWyslania.pose.position.x=0;
			doWyslania.pose.position.y=0;
			doWyslania.pose.position.z=0;
			doWyslania.pose.orientation.w=1;
			doWyslania.pose.orientation.x=0;
			doWyslania.pose.orientation.y=0;
			doWyslania.pose.orientation.z=0;
			pub.publish(doWyslania);
		}
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

			double ox2=kx*sin(fi/2);
			double oy2=ky*sin(fi/2);
			double oz2=kz*sin(fi/2);
			double ow2=cos(fi/2);
			
			xo=ow1*ox2+ox1*ow2+oy1*oz2-oz1*oy2;
			yo=ow1*oy2+oy1*ow2+oz1*ox2-ox1*oz2;
			zo=ow1*oz2+oz1*ow2+ox1*oy2-oy1*ox2;
			wo=ow1*ow2-ox1*ox2-oy1*oy2-oz1*oz2;
		}

	    marker.id++;
	
		
		doWyslania.pose.position.x=x;
		doWyslania.pose.position.y=y;
		doWyslania.pose.position.z=z;
		doWyslania.pose.orientation.w=wo;
		doWyslania.pose.orientation.x=xo;
		doWyslania.pose.orientation.y=yo;
		doWyslania.pose.orientation.z=zo;
	marker.pose=doWyslania.pose;

		pub2.publish(marker);
		pub.publish(doWyslania);
		rate.sleep();
	}

	return 0;
}
