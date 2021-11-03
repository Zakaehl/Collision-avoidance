#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <math.h>
#include "geometry_utils_fd.h"

using namespace std;

const float THRESH_RANGE = 1;	//1m
const float SCALE = 1500;

ros::Publisher output_vel_pub;
geometry_msgs::Twist twist;

tf::TransformListener* listener;

//Salva le velocità in input, inoltrate poi da laserCallback con le eventuali correzioni
void inputVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	float x = msg->linear.x;
	float z = msg->angular.z;
	cerr << "Velocità ricevute: linear.x=" << x << ", angular.z=" << z << endl;
	twist.linear.x = x;
	twist.angular.z = z;
}

//Funzionamento del programma; i punti rilevati dal laser e calcolati rispetto al robot
//esercitano delle forze sul robot, scomposte nelle due componenti dello spazio e sommate tra loro
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	//Agisco se mi muovo
	if (twist.linear.x) {
		//Contengono le due componenti della forza repulsiva totale
		float linear_correction = 0, angular_correction = 0;
		
		tf::StampedTransform transform;
		
		//Calcolo la trasformata tra frame del laser e del robot
		std::string error_msg;
		if (listener->canTransform("/base_footprint", msg->header.frame_id, msg->header.stamp, &error_msg))
			listener->lookupTransform("/base_footprint", msg->header.frame_id, msg->header.stamp, transform);
		Eigen::Isometry2f T = convertPose2D(transform);
		
		//Prendo tutti i punti rilevati dal laser e li calcolo rispetto al frame del robot
		cerr << "Punti a meno di " << THRESH_RANGE << "m rispetto al frame del robot: ";
		for (int i=0; i<msg->ranges.size(); ++i) {
			float range = msg->ranges[i];
			float angle = msg->angle_min + i*msg->angle_increment;
			Eigen::Vector2f p(range*cos(angle), range*sin(angle));
			Eigen::Vector2f transformed_point = T*p;
			
			//Nuove coordinate rispetto al robot
			float x = transformed_point.x();
			float y = transformed_point.y();
			
			//Calcolo la nuova distanza e il nuovo angolo
			range = sqrt(pow(x,2) + pow(y,2));
			angle = atan2(y, x);
			
			//Prendo solo i punti entro la distanza di soglia e davanti se vado avanti,
			//dietro se vado dietro (segni concordi)
			if (range < THRESH_RANGE && (x * twist.linear.x > 0)) {
				cerr << "(" << x << "," << y << ")" << " ";

				//Forza che si oppone pari a 1/(dist)^esp calcolata rispetto alle componenti x e y ("amplificando" sulle y)
				float force_x = (1/pow(range,2)) * cos(angle);
				float force_y = (1/pow(range,4)) * sin(angle);
					
				linear_correction += force_x;
				angular_correction += force_y;
			}
		}
		
		//Scalo le forze e il loro modulo dipende dalla velocità
		linear_correction *= (-1/SCALE) * twist.linear.x;
		angular_correction *= (-1/SCALE) * twist.linear.x;
		cerr << endl << "COMPONENTI DELLA FORZA REPULSIVA SUL ROBOT: (" << linear_correction << "," << angular_correction << ")" << endl;
		
		//Sommo alle velocità ricevute le correzioni
		twist.linear.x += linear_correction;
		twist.angular.z += angular_correction;
	}
	
	cerr << "Velocità inviate: linear.x=" << twist.linear.x << ", angular.z=" << twist.angular.z << endl;
	output_vel_pub.publish(twist);
}

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "collision_avoidance_node");
	
	ros::NodeHandle nh("~");
	string input_vel_topic;
	string laser_topic;
	string output_vel_topic;
	
	nh.param("input_vel_topic", input_vel_topic, string("/cmd_vel_input"));
	nh.param("laser_topic", laser_topic, string("/base_scan"));
	nh.param("output_vel_topic", output_vel_topic, string("/cmd_vel"));
	
	cerr << "Esecuzione con i seguenti parametri: " << endl;
	cerr << "input_vel_topic: " << input_vel_topic << endl;
	cerr << "laser_topic: " << laser_topic << endl;
	cerr << "output_vel_topic: " << output_vel_topic << endl;
	
	ros::Subscriber input_vel_sub = nh.subscribe(input_vel_topic, 10, inputVelCallback);
	ros::Subscriber laser_sub = nh.subscribe(laser_topic, 10, laserCallback);
	
	output_vel_pub = nh.advertise<geometry_msgs::Twist>(output_vel_topic, 10);
	
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;
	
	listener = new tf::TransformListener;
	
	cerr << "Nodo per la collision avoidance avviato" << endl << endl;
	
	ros::spin();
}
