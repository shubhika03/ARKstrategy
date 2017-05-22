#include "../include/strategy/strategy.h"
#include "ros/ros.h"
#include <time.h>


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "strategy");
	strategy STRATEGY;

	STRATEGY.find_herd_bots();

	while(ros::ok())
	{
		STRATEGY.ComputeDistance();
		STRATEGY.FindBotsInsideCircle();
		STRATEGY.FirstOperation();
		STRATEGY.t_plan();
		
		while(STRATEGY.IsOutsideWhite())
		{
			time_t timer1, timer2;
			timer1 = time(NULL);
			STRATEGY.FirstOperation();
			STRATEGY.FindBotsInsideCircle();
			STRATEGY.t_plan();
			timer2 = time(NULL) - timer1;
			while(timer2<=10)
				timer2 = time(NULL) - timer1;

		}
	}
}
