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

using namespace std;

typedef struct {
	int x;
	int y;
}point;

class strategy{

	public:
		void find_herd_bots();                                   // finds the bot to be herded in the first 20 secs
		void herd_bots();                                        // herds the bots for the first 20 secs
		void ComputeDistance();                                 // computes distance from green line
		void FindBotsInsideCircle();                             // finds bot inside the circle
		void FirstOperation();                             // decides the operation to be performed on the target bot
		float angle(float ang);                                  // to make sure the angle is within the range
	    float dist_whitel();                                      // calculates the least dist from the white line
	    void action(int bot_no);                                 // action to be performed on the bots inside the circle
	    void t_plan();                                           // formulates the plan for the bot inside the circle
	    void GetEulerAngles(double* yaw, double* pitch, double* roll);
	    void posecallback(nav_msgs::Odometry::ConstPtr msg);
	    void centercallback(nav_msgs::Odometry::ConstPtr msg);
	    void toQuaternion(double pitch, double roll, double yaw);
			int IsOutsideWhite();                                   //check is the center bot is outside green line
		void quadcallback(nav_msgs::Odometry::ConstPtr msg);
	    ros::NodeHandle n;
	    ros::Publisher pub;
	    ros::Subscriber sub_quad;
	    ros::Subscriber sub;
	    Go2Goal obj;

	private:
		typedef pair <double, int> p;
		set<p> ClosestBot;                                      //  set containing bot id and dist from green line
		vector<int> BotsInsideCircle;                           //  vector containing id of bots inside the circle
		double centerX, centerY,centerZ;                                //  x and y coordinates of the center/target bot
		double posX, posY,posZ,quadX,quadY,quadZ;               //  x and y coordinates of the the other bots inside the circle
		double x, y, z, w;                                      //  needed for qauternian angle to euler angle
		vector <float>distance_bots;
		int no1,no2;
};

#endif
