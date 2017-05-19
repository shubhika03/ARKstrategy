#ifndef STRATEGY_H
#define STRATEGY_H

#include <iostream>
#include <time.h>
#include <math.h>
#include <utility>
#include <set>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ark_llp/go2goal.h"
#include "flyquad.h"

using namespace std;

typedef struct
{
	double x,y,z,w;
}qt;

class strategy{

	public:
    void rotate (double relative_angle, char publish_name[40], int ID);
		void find_herd_bots();                                  																											 // finds the bot to be herded in the first 20 secs
		void herd_bots();                                     																											   // herds the bots for the first 20 secs
		/*
		void ComputeDistance();                    																												             // computes distance from green line
		void FindBotsInsideCircle();                            																											 // finds bot inside the circle
		void FirstOperation();                            																														 // decides the operation to be performed on the target bot
		float angle(float ang);                                  																											 // to make sure the angle is within the range
    	float dist_whitel();                                   																											   // calculates the least dist from the white line
    	void action(int bot_no);                               																											   // action to be performed on the bots inside the circle
    	void t_plan();
		*/ 
		float angle(float ang);                                                      																											   // formulates the plan for the bot inside the circle
    	void GetEulerAngles(qt q,double* yaw, double* pitch, double* roll);
		//int IsOutsideWhite();                                   																											 //check is the center bot is outside green line
		void retrieve_pose(int ID, nav_msgs::Odometry *gbpose);
		void groundbot4Callback(const nav_msgs::Odometry::ConstPtr& msg);
    	void groundbot5Callback(const nav_msgs::Odometry::ConstPtr& msg);
    	void groundbot6Callback(const nav_msgs::Odometry::ConstPtr& msg);
    	void groundbot7Callback(const nav_msgs::Odometry::ConstPtr& msg);
    	void groundbot8Callback(const nav_msgs::Odometry::ConstPtr& msg);
    	void groundbot9Callback(const nav_msgs::Odometry::ConstPtr& msg);
    	void groundbot10Callback(const nav_msgs::Odometry::ConstPtr& msg);
    	void groundbot11Callback(const nav_msgs::Odometry::ConstPtr& msg);
    	void groundbot12Callback(const nav_msgs::Odometry::ConstPtr& msg);
    	void groundbot13Callback(const nav_msgs::Odometry::ConstPtr& msg);

		tap_n_turn fly_quad;

   	strategy()
  	{
		clockwise = false;
		angular_speed = 0.78;
	    sub_4 = n.subscribe("robot4/odom", 100, &strategy::groundbot4Callback,this);
	    sub_5 = n.subscribe("robot5/odom", 100, &strategy::groundbot5Callback,this);
	    sub_6 = n.subscribe("robot6/odom", 100, &strategy::groundbot6Callback,this);
	    sub_7 = n.subscribe("robot7/odom", 100, &strategy::groundbot7Callback,this);
	    sub_8 = n.subscribe("robot8/odom", 100, &strategy::groundbot8Callback,this);
	    sub_9 = n.subscribe("robot9/odom", 100, &strategy::groundbot9Callback,this);
	    sub_10 = n.subscribe("robot10/odom", 100, &strategy::groundbot10Callback,this);
	    sub_11 = n.subscribe("robot11/odom", 100, &strategy::groundbot11Callback,this);
	    sub_12 = n.subscribe("robot12/odom", 100, &strategy::groundbot12Callback,this);
	    sub_13 = n.subscribe("robot13/odom", 100, &strategy::groundbot13Callback,this);

	  }

	ros::NodeHandle n;
    ros:: Subscriber sub_4;
    ros:: Subscriber sub_5;
    ros:: Subscriber sub_6;
    ros:: Subscriber sub_7;
    ros:: Subscriber sub_8;
    ros:: Subscriber sub_9;
    ros:: Subscriber sub_10; 
    ros:: Subscriber sub_11;
    ros:: Subscriber sub_12;
    ros:: Subscriber sub_13;
    ros:: Publisher publi;


	private:
		typedef pair <double, int> p;
		set<p> ClosestBot;                                      //  set containing bot id and dist from green line
		vector<int> BotsInsideCircle;                           //  vector containing id of bots inside the circle
		double centerX, centerY,centerZ;                                //  x and y coordinates of the center/target bot
		double angular_speed;                                      //  needed for qauternian angle to euler angle
		vector <float>distance_bots;
		int no1,no2;
        bool clockwise;

		

	  nav_msgs::Odometry gb4pose;                                     //position of ground bot
	  nav_msgs::Odometry gb5pose;
	  nav_msgs::Odometry gb6pose;
	  nav_msgs::Odometry gb7pose;
	  nav_msgs::Odometry gb8pose;
	  nav_msgs::Odometry gb9pose;
	  nav_msgs::Odometry gb10pose;
	  nav_msgs::Odometry gb11pose;
	  nav_msgs::Odometry gb12pose;
	  nav_msgs::Odometry gb13pose;
};

#endif
