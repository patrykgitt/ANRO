#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "zadanie4/jint_control_srv.h"
#include <math.h>
#include <visualization_msgs/Marker.h>

double const PI=M_PI;
double teta1_0=0, teta2_0=0, d3_0=1.65; // Wartości początkowe
double teta1=0.0, teta2=0.0, d3=1.65; // Wartości aktualne
double teta1zad=0,teta2zad=0,d3zad=1.65; // Wartości zadane przez klienta
double f=10; // Częstotliwość działania programu
double Tp=1/f; // Okres próbkowania
double joint2_lower=-2.9,joint2_upper=2.9,joint3_lower=0.3,joint3_upper=3.0; // Dane domyślne - ograniczenia kinematyki
int n=0; // Liczba iteracji pętli
int k=0; // Indeks iteracji pętli
double czas;
bool czyLiniowa=true; // Domyślnie stosujemy interpolację liniową
bool czyInterpolujemy=false;

// Procedura do obsługi żądania
bool procedura(zadanie4::jint_control_srv::Request &req, zadanie4::jint_control_srv::Response &res)
{
	if(req.czasRuchu <= 0)
	{
		// Wysłanie odpowiedzi negatywnej do klienta
		res.rezultat="Podaj dodatni czas trwania ruchu.";
		ROS_WARN("Podano nieprawidlowy czas ruchu.");
		return true;
	}
	
	
	if(req.pozycjaZadana2 > joint2_upper || req.pozycjaZadana2 < joint2_lower)
		{	
			res.rezultat="Podano niedozwolone wartosci polozenia (pozycja nr 2).";
			ROS_WARN("Wyznaczenie polozenia nie jest mozliwe.");
			return true;
		}
	if(req.pozycjaZadana3 > joint3_upper || req.pozycjaZadana3 < joint3_lower)
		{
			res.rezultat="Podano niedozwolone wartosci polozenia (pozycja nr 3).";
			ROS_WARN("Wyznaczenie polozenia nie jest mozliwe.");
			return true;
		}	

	// Zapisanie wartości zadanych położenia stawów
	teta1zad=req.pozycjaZadana1;
	if(teta1zad>PI) // Ograniczamy wartości do przedziału [-PI;PI]
		while(teta1zad>PI)
			teta1zad-=2*PI;
	else if(teta1zad<-PI)
		while(teta1zad<-PI)
			teta1zad+=2*PI;
		
	teta2zad=req.pozycjaZadana2;
	d3zad=req.pozycjaZadana3;
	// Jeżeli program znajdzie się w tym miejscu, to znaczy, że podano prawidłowe wartości
	czyInterpolujemy=true;
	// Zapisanie wartości ,,startowych"
	teta1_0=teta1;
	teta2_0=teta2;
	d3_0=d3;

	czas=req.czasRuchu;
	n=czas/Tp; 
	k=1;
	if(req.rodzaj=="nieliniowa") // Porównanie napisów
	{
		// Jeżeli wybieramy krzywe sklejane
		czyLiniowa=false;
		res.rezultat="Zastosowano interpolacje nieliniowa";
	}
	else // Domyślnie tosujemy interpolację liniową
	{
		czyLiniowa=true;
		res.rezultat="Zastosowano interpolacje liniowa";	
	}

	return true;
}

int main(int argc, char **argv)
{
	// Inicjalizacja ros-a
	ros::init(argc,argv,"jint");
	ros::NodeHandle nh;
	ros::Publisher pub=nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
	ros::ServiceServer serwer = nh.advertiseService("jint_control_srv",procedura); // Rodzaj serwera
ros::Publisher pub2=nh.advertise<visualization_msgs::Marker>("/visualization_marker", 100);	
visualization_msgs::Marker marker;
marker.id = 0;
	ros::Rate rate(f);

	// Nadanie wartości początkowych zmiennym;
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
		ros::spinOnce();
		if(n==0 && k==1)
		{
			teta1=teta1zad;
			teta2=teta2zad;
			d3=d3zad;
		}

		else if(k<1 || n < 1)
		{
			czyInterpolujemy=false;
		}
		
		// Obliczenie wartości położenia
		if(czyLiniowa && czyInterpolujemy) // Interpolacja liniowa
		{
			teta1=teta1_0+((teta1zad-teta1_0)/czas)*k*Tp;
			teta2=teta2_0+((teta2zad-teta2_0)/czas)*k*Tp;
			d3=d3_0+((d3zad-d3_0)/czas)*k*Tp;
		}
		else if(!czyLiniowa && czyInterpolujemy) // Interpolacja nieliniowa
		{
			/*double a=(teta1zad-teta1_0)/(czas*czas*czas);
			teta1=teta1_0+3*k*Tp*a*czas*czas-3*k*k*Tp*Tp*czas*a+k*k*k*Tp*Tp*Tp*a;
			a=(teta2zad-teta2_0)/(czas*czas*czas);
			teta2=teta2_0+3*k*Tp*a*czas*czas-3*k*k*Tp*Tp*czas*a+k*k*k*Tp*Tp*Tp*a;
			a=(d3zad-d3_0)/(czas*czas*czas);
			d3=d3_0+3*k*Tp*a*czas*czas-3*k*k*Tp*Tp*czas*a+k*k*k*Tp*Tp*Tp*a;*/

			double a=-2*(teta1zad-teta1_0)/(czas*czas*czas);
			double b=3*(teta1zad-teta1_0)/(czas*czas);
			teta1=teta1_0+a*k*k*k*Tp*Tp*Tp+b*k*k*Tp*Tp;
			a=-2*(teta2zad-teta2_0)/(czas*czas*czas);
			b=3*(teta2zad-teta2_0)/(czas*czas);	
			teta2=teta2_0+a*k*k*k*Tp*Tp*Tp+b*k*k*Tp*Tp;
			a=-2*(d3zad-d3_0)/(czas*czas*czas);
			b=3*(d3zad-d3_0)/(czas*czas);	
			d3=d3_0+a*k*k*k*Tp*Tp*Tp+b*k*k*Tp*Tp;

			
		}

		k++;
		if (k>=n)
		{	
			teta1=teta1zad;
			teta2=teta2zad;
			d3=d3zad;
			n=0;
			k=0;
			czyInterpolujemy=false;
		}
		
		sensor_msgs::JointState doWyslania;
		// Odmierzanie czasu
		doWyslania.header.stamp=ros::Time::now();
		// Tworzenie wiadomości do wysłania	
		doWyslania.header.frame_id="";
		doWyslania.position.push_back(teta1); // Dodaje element na końcu wektora
		doWyslania.position.push_back(teta2);
		doWyslania.position.push_back(d3);
		doWyslania.name.push_back("joint1");
		doWyslania.name.push_back("joint2");
		doWyslania.name.push_back("joint3");

 marker.header.frame_id = "/base_link";
    marker.ns = "final_pos";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.id++;
marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.5;
        marker.color.a = 0.8;
double x = 3*cos(PI/5+teta1-teta2)+cos(teta1+PI/5);
double y = 3*sin(PI/5+teta1-teta2)+sin(PI/5+teta1);
double z = -d3;
marker.pose.position.x=x;
marker.pose.position.y=y;
marker.pose.position.z=z;	
pub2.publish(marker);
		pub.publish(doWyslania);
		rate.sleep();		
	}
	return 0;
}
