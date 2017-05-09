#ifndef STRATEGY_H
#define STRATEGY_H

#include <iostream>
#include <time.h>
#include <math.h>
#include <utility>
#include <set>
#include "ros/ros.h"

using namespace std;

typedef struct {
	int x;
	int y;
}point;

class strategy{

	public:
		void find_herd_bots();                                   // finds the bot to be herded in the first 20 secs
		void herd_bots();                                        // herds the bots for the first 20 secs
		void ComputeDistance(double y, int BotID);               // computes distance from green line
		void FindBotsInsideCircle();                             // finds bot inside the circle
		void FirstOperation(int centerBotID);                    // decides the operation to be performed on the target bot
		float angle(float ang);                                  // to make sure the angle is within the range
	    float distwhitel(point bot,float orient);                // calculates the least dist from the white line
	    void action();         									 // action to be performed on the bots inside the circle
	    void t_plan(int no);                                     // formulates the plan for the bot inside the circle
	    void GetEulerAngles(double w, double x, double y, double z, double* yaw, double* pitch, double* roll);
	    void posecallback(nav_msgs::Odometry::ConstPtr& msg);
	    void centercallback(nav_msgs::Odometry::ConstPtr& msg)
			int IsOutsideWhite();
	    ros::NodeHandle n;
	    ros::Publisher pub;
	    ros::Suscriber sub;

	private:
		int counter = 0;
		typedef Pair <double, int> p;
		set<p> ClosestBot;                                      //  set containing bot id and dist from green line
		vector<int> BotsInsideCircle;                           //  vector containing id of bots inside the circle
		double centerX, centerY;                                //  x and y coordinates of the center/target bot
		double posX, posY;                                      //  x and y coordinates of the the other bots inside the circle
		double x, y, z, w;                                      //  needed for qauternian angle to euler angle
		vector <float>distance_bots;
		int no1,no2;
};

#endif
