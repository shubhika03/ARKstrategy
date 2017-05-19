#include <time.h>                                     //confirm x and y axis from Aditi
#include <math.h>
#include <utility>
#include <set>
#include <vector>
#include "nav_msgs/Odometry.h"
#include "../include/strategy/strategy.h"
#include "ros/ros.h"
#include "ark_llp/go2goal.h"
#include "../include/strategy/flyquad.h"

#define PI 3.14159
#define MIN_DIST 9
using namespace std;


void strategy:: retrieve_pose(int ID, nav_msgs::Odometry *gbpose)
{
  switch(ID)
  {
    case 4:
      gbpose->pose.pose.orientation.x = gb4pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb4pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb4pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb4pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb4pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb4pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb4pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb4pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb4pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb4pose.twist.twist.linear.z;
      //return gbpose;
      break;

    case 5:
      gbpose->pose.pose.orientation.x = gb5pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb5pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb5pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb5pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb5pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb5pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb5pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb5pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb5pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb5pose.twist.twist.linear.z;
     // return gbpose;
      break;

    case 6:
      gbpose->pose.pose.orientation.x = gb6pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb6pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb6pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb6pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb6pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb6pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb6pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb6pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb6pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb6pose.twist.twist.linear.z;
     // return gbpose;
      break;
    case 7:
      gbpose->pose.pose.orientation.x = gb7pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb7pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb7pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb7pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb7pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb7pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb7pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb7pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb7pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb7pose.twist.twist.linear.z;
     // return gbpose;
      break;

    case 8:
    gbpose->pose.pose.orientation.x = gb8pose.pose.pose.orientation.x;
    gbpose->pose.pose.orientation.y = gb8pose.pose.pose.orientation.y;
    gbpose->pose.pose.orientation.z = gb8pose.pose.pose.orientation.z;
    gbpose->pose.pose.orientation.w = gb8pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb8pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb8pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb8pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb8pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb8pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb8pose.twist.twist.linear.z;
     // return gbpose;
      break;

    case 9:
    gbpose->pose.pose.orientation.x = gb9pose.pose.pose.orientation.x;
    gbpose->pose.pose.orientation.y = gb9pose.pose.pose.orientation.y;
    gbpose->pose.pose.orientation.z = gb9pose.pose.pose.orientation.z;
    gbpose->pose.pose.orientation.w = gb9pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb9pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb9pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb9pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb9pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb9pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb9pose.twist.twist.linear.z;
     // return gbpose;
      break;

    case 10:
    gbpose->pose.pose.orientation.x = gb10pose.pose.pose.orientation.x;
    gbpose->pose.pose.orientation.y = gb10pose.pose.pose.orientation.y;
    gbpose->pose.pose.orientation.z = gb10pose.pose.pose.orientation.z;
    gbpose->pose.pose.orientation.w = gb10pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb10pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb10pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb10pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb10pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb10pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb10pose.twist.twist.linear.z;
      //return gbpose;
      break;

    case 11:
    gbpose->pose.pose.orientation.x = gb11pose.pose.pose.orientation.x;
    gbpose->pose.pose.orientation.y = gb11pose.pose.pose.orientation.y;
    gbpose->pose.pose.orientation.z = gb11pose.pose.pose.orientation.z;
    gbpose->pose.pose.orientation.w = gb11pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb11pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb11pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb11pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb11pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb11pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb11pose.twist.twist.linear.z;
      //return gbpose;
      break;

    case 12:
    gbpose->pose.pose.orientation.x = gb12pose.pose.pose.orientation.x;
    gbpose->pose.pose.orientation.y = gb12pose.pose.pose.orientation.y;
    gbpose->pose.pose.orientation.z = gb12pose.pose.pose.orientation.z;
    gbpose->pose.pose.orientation.w = gb12pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb12pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb12pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb12pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb12pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb12pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb12pose.twist.twist.linear.z;
      //return gbpose;
      break;

    case 13:
      gbpose->pose.pose.orientation.x = gb13pose.pose.pose.orientation.x;
      gbpose->pose.pose.orientation.y = gb13pose.pose.pose.orientation.y;
      gbpose->pose.pose.orientation.z = gb13pose.pose.pose.orientation.z;
      gbpose->pose.pose.orientation.w = gb13pose.pose.pose.orientation.w;
      gbpose->pose.pose.position.x = gb13pose.pose.pose.position.x;
      gbpose->pose.pose.position.y = gb13pose.pose.pose.position.y;
      gbpose->pose.pose.position.z = gb13pose.pose.pose.position.z;
      gbpose->twist.twist.linear.x = gb13pose.twist.twist.linear.x;
      gbpose->twist.twist.linear.y = gb13pose.twist.twist.linear.y;
      gbpose->twist.twist.linear.z = gb13pose.twist.twist.linear.z;
      //return gbpose;
      break;
  }
}


void strategy::find_herd_bots()
{
	float max;
	for(int i=4;i<14;i++)
  {
    nav_msgs:: Odometry temp;
    ros::Rate loop_rate(10);
    ros::spinOnce();
    loop_rate.sleep();
    retrieve_pose(i, &temp);
  	distance_bots.push_back(temp.pose.pose.position.x);
	}

	max = -FLT_MAX;
	no1 = no2 = 4;
	for(int i=0;i<10;i++)
  {
		if(distance_bots[i]<0)
    {
			if(distance_bots[i]>=max || max-distance_bots[i]<0.1)
      {
				no2 = no1;
				max=distance_bots[i];
				no1 = i+4;
			}
		}
	}
	herd_bots();
}

void strategy::herd_bots()                              //go to the no1 and no2 bot and tap for turning them towards the center
{
	int bots[2] = {no1, no2};
	ROS_INFO("%d  %d\n",no1,no2);
	int z=0;
	while(z<=1)
	{
		char publish_name[40];
    sprintf(publish_name, "robot%d/cmd_vel", bots[z]);

    //fly_quad.navigate_quad(bots[z]);

		rotate(PI,publish_name,bots[z]);
    z++;
	}
}

/*
void strategy::ComputeDistance(){                                     //computes distance of every bot from the green line

	for(int i=4;i<14;i++){

		char topic_name[40];
		sprintf(topic_name, "robot%d/odom", i);
		sub = n.subscribe(topic_name, 1000, &strategy::posecallback,this);
		ros::Rate loop_rate(10);
		posX=0;
		while(posX == 0.0){
			ros::spinOnce();
			loop_rate.sleep();
			//ROS_INFO("%f\n",posX);
		}



		//ros::spinOnce();
		double y = 10 - posX;
		//ROS_INFO("x = %f", y);
		int BotID = i;
		ClosestBot.insert(make_pair(y,BotID));

	}
}


void strategy::FindBotsInsideCircle(){                               //to find the bot inside the 5m circle

	int centerBotID;

	p temp = *ClosestBot.begin();
	centerBotID = temp.second;
	ROS_INFO("center bot id %d\n",centerBotID);
	int count=4;

	while(count<14){

		if(count!=centerBotID){

			char topic_name[40];
			sprintf(topic_name, "robot%d/odom", centerBotID);
			sub = n.subscribe(topic_name, 1, &strategy::centercallback,this);

			ros::Rate loop_rate(10);
		    centerX=0;
			while(centerX == 0.0){
			ros::spinOnce();
			loop_rate.sleep();
			}


			//ros::spinOnce();
			char topic_name2[40];
			sprintf(topic_name2, "robot%d/odom", count);
			sub = n.subscribe(topic_name2, 1, &strategy::posecallback,this);

			ros::Rate loop_rate2(10);
		    posX=0;
		    while(posX == 0.0){
			ros::spinOnce();
			loop_rate2.sleep();

		    }


			//ros::spinOnce();
		}

		double dist=pow(-posY+centerY,2) + pow(posX-centerX,2);
		//ROS_INFO("%f\n",dist);
		if(dist - 25 <= 0 && count!=centerBotID){
			//ROS_INFO("BOT ID: %d inside circle\n",count);
			BotsInsideCircle.push_back(count);
		}
		count++;
	}
}

void strategy::FirstOperation(){                                          //to decide the first operation of the center bot

	double yaw=0, pitch=0, roll=0;

	int centerBotID;

	p temp = *ClosestBot.begin();
	centerBotID = temp.second;

	char topic_name[40],publish_name[40];

	sprintf(topic_name, "robot%d/odom", centerBotID);
        sprintf(publish_name, "robot%d/cmd_vel",  centerBotID);
	sub = n.subscribe(topic_name, 1000, &strategy::centercallback,this);
        ros::spinOnce();

		while(ros::ok())
		{
			if(fabs(centerX)<=10 && fabs(centerY)<=10)
			{
				obj.set_dest(-centerY, centerX, centerZ+0.7, yaw);
				sub_quad = n.subscribe("ground_truth/state", 1000, &strategy::quadcallback, this);
				sub = n.subscribe(topic_name, 1000, &strategy::centercallback,this);
				if(abs(quadX-centerX)<=0.1 && abs(quadY-centerY)<=0.1 && abs(quadZ-centerZ)<=0.8)
					break;
			}
		}

	double theta1 = atan((10 - centerX)/(10 + centerY));
	double theta2 = atan((10 - centerX)/(-10 + centerY));

	GetEulerAngles(&yaw, &pitch, &roll);

	ROS_INFO("theta1: %f\n", theta1);
	ROS_INFO("theta2: %f\n", theta2);
	ROS_INFO("Yaw: %f\n", yaw);

	if(yaw>=theta2 && yaw<=theta1)
		ROS_INFO("Condition 0\n");

	else if((yaw>=angle(theta1+PI) && yaw<=angle(theta2+PI) ) || (angle(theta1+PI)*angle(theta2+PI)<0 && (yaw>=angle(theta1+PI) || yaw<=angle(theta2+PI)) ) )
	{
		ROS_INFO("Condition 1 %lf\n",yaw);
		//180 degree turn, come in front
		yaw = angle(yaw + PI);

      rotate(yaw,publish_name,topic_name);
      sub = n.subscribe(topic_name, 1000, &strategy::posecallback,this);
            GetEulerAngles(&yaw, &pitch, &roll);
            ROS_INFO("After rotation: %f\n",yaw);
	}
	else if((yaw>theta2 && yaw<angle(theta2+PI/4)) || ( theta2*angle(theta2+PI/4)<0 && (yaw>theta2 || yaw<angle(theta2+PI/4))))
	{
		//45 degree turn, tap from top
		ROS_INFO("Condition 2 \n");
		yaw = angle(yaw-PI/4);
		/*toQuaternion(yaw,pitch,roll);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;

    rotate(yaw,publish_name,topic_name);
    sub = n.subscribe(topic_name, 1000, &strategy::posecallback,this);
            GetEulerAngles(&yaw, &pitch, &roll);
            ROS_INFO("After rotation: %f \n",yaw);
	}
	else if(yaw>=angle(theta2 + PI/4) && yaw<=angle(theta2 + PI/2) || ( angle(theta2+PI/4)*angle(theta2+PI/2)<0 && (yaw>angle(theta1+PI/4) || yaw<angle(theta2+PI/2))))
	{
		//90 degree turn, tap twice
		ROS_INFO("Condition 3  \n");
		yaw = angle(yaw-PI/2);
		/*toQuaternion(yaw,pitch,roll);
//		obj.set_dest(centerX, centerY, centerZ, yaw);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;

    rotate(yaw,publish_name,topic_name);
    sub = n.subscribe(topic_name, 1000, &strategy::posecallback,this);
            GetEulerAngles(&yaw, &pitch, &roll);
            ROS_INFO("After rotation: \n");

	}
	else if(yaw>=angle(theta2+PI) && yaw<=angle(theta2+PI+PI/4) || (angle(theta2+PI)*angle(theta2+PI+PI/4) && (yaw>=angle(theta2+PI) && yaw<=angle(theta2+PI+PI/4))))
	{
		//first rotate by 180 degrees and then turn by 45 degrees
		ROS_INFO("Condition 4 \n");
		yaw = angle(yaw-PI-PI/4);
		/*toQuaternion(yaw,pitch,roll);
//		obj.set_dest(centerX, centerY, centerZ, yaw);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;

    rotate(yaw,publish_name, topic_name);
    sub = n.subscribe(topic_name, 1000, &strategy::posecallback,this);
            GetEulerAngles(&yaw, &pitch, &roll);
            ROS_INFO("After rotation: %f\n", yaw);
	}
}

*/





void strategy::rotate (double relative_angle, char publish_name[40], int ID)
{
	geometry_msgs::Twist vel_msg;
  publi = n.advertise<geometry_msgs::Twist>(publish_name,1000);
  //set a random linear velocity in the x-axis
	vel_msg.linear.x =0;
  vel_msg.linear.y =0;
  vel_msg.linear.z =0;
  //set a random angular velocity in the y-axis
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;

  if (clockwise)
 	  vel_msg.angular.z =-abs(angular_speed);
  else
 	  vel_msg.angular.z =abs(angular_speed);

  ros::Rate loop_rate(100);
  double Yaw,Pitch,Roll,yaw_i,pitch_i,roll_i;
  Yaw=Pitch=Roll= 0.0;
  nav_msgs::Odometry temp;

  ros::spinOnce();
  retrieve_pose(ID, &temp);
  loop_rate.sleep();
  qt q;
  q.x = temp.pose.pose.orientation.x;
  q.y = temp.pose.pose.orientation.y;
  q.z = temp.pose.pose.orientation.z;
  q.w = temp.pose.pose.orientation.w;
  GetEulerAngles(q, &yaw_i, &pitch_i, &roll_i);
  ROS_INFO("before %f\n",yaw_i);

  while(1)
  {

    ros::spinOnce();
    retrieve_pose(ID, &temp);
    loop_rate.sleep();
    qt q;
    q.x = temp.pose.pose.orientation.x;
    q.y = temp.pose.pose.orientation.y;
    q.z = temp.pose.pose.orientation.z;
    q.w = temp.pose.pose.orientation.w;
    GetEulerAngles(q, &Yaw, &Pitch, &Roll);
    
    publi.publish(vel_msg);
    if(fabs(angle(Yaw) - angle(yaw_i+relative_angle)) <= 0.1)
  	 break;

}

ROS_INFO("after %f\n",Yaw);
 vel_msg.angular.z = 0;
 publi.publish(vel_msg);
}


/*

int strategy::IsOutsideWhite()
{
	int centerBotID;
	p temp = *ClosestBot.begin();
	centerBotID = temp.second;
	char topic_name[40];
	sprintf(topic_name, "robot%d/odom", centerBotID);
	ros::Rate loop_rate(10);
    sub = n.subscribe(topic_name, 1, &strategy::centercallback,this);
	ros::spinOnce();
	loop_rate.sleep();
	if(centerY>=MIN_DIST)
		return 1;
	else
		return 0;
}

*/

void strategy::GetEulerAngles(qt q, double* yaw, double* pitch, double* roll)
   {
       const double w2 = q.w*q.w;
       const double x2 = q.x*q.x;
       const double y2 = q.y*q.y;
       const double z2 = q.z*q.z;
       const double unitLength = w2 + x2 + y2 + z2;    // Normalised == 1, otherwise correction divisor.
       const double abcd = q.w*q.x + q.y*q.z;
       const double eps = 1e-7;    // TODO: pick from your math lib instead of hardcoding.
       const double pi = 3.14159265358979323846;   // TODO: pick from your math lib instead of hardcoding.
       if (abcd > (0.5-eps)*unitLength)
       {
           *yaw = 2 * atan2(q.y, q.w);
           *pitch = pi;
           *roll = 0;
       }
       else if (abcd < (-0.5+eps)*unitLength)
       {
           *yaw = -2 * ::atan2(q.y, q.w);
           *pitch = -pi;
           *roll = 0;
       }
       else
       {
           const double adbc = q.w*q.z - q.x*q.y;
           const double acbd = q.w*q.y - q.x*q.z;
           *yaw = ::atan2(2*adbc, 1 - 2*(z2+x2));
           *pitch = ::asin(2*abcd/unitLength);
           *roll = ::atan2(2*acbd, 1 - 2*(y2+x2));
       }
   }

/*

  float strategy::dist_whitel(){

  	double yaw,pitch,roll;
  	GetEulerAngles(&yaw,&pitch,&roll);
  	float orient=yaw;


  if(orient>=0 && orient<PI/2){
      return 10-posY;
  }

  if(orient>=PI/2 && orient>PI){
      return 10+posY;
  }

  if(orient<0 && orient>-PI/2){
  	if(10-posX<10+posY)
      return 10-posX;
  	else
  		return 10+posY;
  }

  if(orient>-PI && orient<-PI/2){
  	if(10+posY<10+posX)
      return 10+posY;
  	else
  		return 10+posX;
  }

}*/

float strategy::angle(float ang){

  if(ang>=-1*PI && ang<=PI)
    return ang;
  else if(ang>PI)
    return ang-2*PI;
  else if(ang<-PI)
    return ang+2*PI;

}


/*void strategy::action(int bot_no){
  float theta1,theta2,orient;
  char topic_name[40],publish_name[40];
  sprintf(topic_name, "robot%d/odom", bot_no);
  //sprintf(publish_name, "robot%d/cmd_vel", bot_no);

  theta1=angle(atan((centerX-posX)/(centerY-posY))-PI/4);
  theta2=angle(theta1 + PI/2);

  double yaw,pitch,roll;
  GetEulerAngles(&yaw,&pitch,&roll);
  orient=yaw;


  pub = n.advertise<nav_msgs::Odometry>(topic_name, 1);

  nav_msgs::Odometry msg;
  msg.pose.pose.position.x = posY;
  msg.pose.pose.position.y = -posX;

  ROS_INFO("theta 1 action%f\n",theta1);
  ROS_INFO("theta 2 action  %f\n",theta2);
  ROS_INFO("yaw %f\n",yaw);


		while(ros::ok())
		{
			obj.set_dest(-posY, posX, posZ, yaw);
			sub_quad = n.subscribe("ground_truth/state", 1000, &strategy::quadcallback, this);
			sub = n.subscribe(topic_name, 1000, &strategy::posecallback,this);
			if(abs(quadX-posX)<=1 && abs(quadY-posY)<=1 && abs(quadZ-posZ)<=1)
				break;
		}

  if( (orient>theta2 && orient<angle(theta2+PI/4)) || ( theta2*angle(theta2+PI/4)<0 && (orient>theta2 || orient<angle(theta2+PI/4) ) ) ){
    //one 45 degree rotation anticlockwise
  	yaw=angle(yaw+PI/4);
  	ROS_INFO("condition 1\n");
    toQuaternion(yaw,pitch,roll);
    //obj.set_dest(posX, posY, posZ, yaw);
   msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
  //  pub.publish(msg);
  //rotate(yaw,publish_name);
  }
  else if( (orient>=angle(theta1+PI) && orient<=angle(theta2+PI) ) || (angle(theta1+PI)*angle(theta2+PI)<0 && (orient>=angle(theta1+PI) || orient<=angle(theta2+PI)) ) ){
    //turn 180 degree
    //yaw=angle(yaw+PI);
    ROS_INFO("condition 2\n");
    toQuaternion(yaw,pitch,roll);
    //obj.set_dest(posX, posY, posZ, yaw);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
   // pub.publish(msg);
   //rotate(yaw,publish_name);
  }
  else if( (orient<theta1 && orient>=angle(theta1-PI/4)) || ( theta1*angle(theta1-PI/4)<0 && (orient>angle(theta1-PI/4) || orient<theta1 ) ) ){
    //one 180 degree turn and then one 45 degree anti clockwise rotation
    //yaw=angle(yaw+5*PI/4);
    ROS_INFO("condition 3\n");
    toQuaternion(yaw,pitch,roll);
    //obj.set_dest(posX, posY, posZ, yaw);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
    //pub.publish(msg);
  }
  else if( (orient>angle(theta2+PI/4) && orient<=angle(theta2+PI/2)) || ( angle(theta2+PI/4)*angle(theta2+PI/2)<0 && (orient>angle(theta1+PI/4) || orient<angle(theta2+PI/2) ) )){
    //two 45 degree anticlockwise rotations
    //yaw=angle(yaw+PI/2);
    ROS_INFO("condition 4\n");
    toQuaternion(yaw,pitch,roll);
    //obj.set_dest(posX, posY, posZ, yaw);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
    //pub.publish(msg);
  }
  else if( (orient<angle(theta1-PI/4) && orient>=angle(theta1-PI/2)) || ( angle(theta1-PI/2)*angle(theta1-PI/4)<0 && (orient>angle(theta1-PI/2) || orient<angle(theta1-PI/4) ) )){
    //one 180 degree turn and two 45 degree anticlockwise turn
    //yaw=angle(yaw+3*PI/2);
    ROS_INFO("condition 5\n");
    toQuaternion(yaw,pitch,roll);
    //obj.set_dest(posX, posY, posZ, yaw);
    msg.pose.pose.orientation.x=x;
    msg.pose.pose.orientation.y=y;
    msg.pose.pose.orientation.z=z;
    msg.pose.pose.orientation.w=w;
    //pub.publish(msg);
  //rotate(yaw,publish_name);
  }

}


void strategy::t_plan()
{

//  ROS_INFO("t_plan started\n");
  int centerBotID;
  p temp = *ClosestBot.begin();
  centerBotID = temp.second;

  int size=BotsInsideCircle.size();

  char topic_name[40];
  sprintf(topic_name, "robot%d/odom", centerBotID);
  sub = n.subscribe(topic_name, 1, &strategy::centercallback,this);
  ros::spinOnce();                                           // suscribe to the publisher to get the coordinates of the target bot

  float dis,min;
  int bot_no=BotsInsideCircle[0];
  char topic_name2[40];

  for(int i=0;i<size;i++){

	sprintf(topic_name2, "robot%d/odom", BotsInsideCircle[i]);
	sub = n.subscribe(topic_name2, 1, &strategy::posecallback,this);
	ros::spinOnce();

    dis=dist_whitel();
    if(i==0)
      min=dis;
    else if(dis<min){
      min=dis;
      bot_no=BotsInsideCircle[i];
    }

  }
  ROS_INFO("bot inside the circle to be considered %d\n",bot_no);
  sprintf(topic_name2, "robot%d/odom", bot_no);
  sub = n.subscribe(topic_name2, 1, &strategy::posecallback,this);
  ros::spinOnce();

  action(bot_no);
}
*/
void strategy::groundbot4Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb4pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb4pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb4pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb4pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb4pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb4pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb4pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb4pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategy::groundbot5Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb5pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb5pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb5pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb5pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb5pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb5pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb5pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb5pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategy::groundbot6Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb6pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb6pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb6pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb6pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb6pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb6pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb6pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb6pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategy::groundbot7Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb7pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb7pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb7pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb7pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb7pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb7pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb7pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb7pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategy::groundbot8Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb8pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb8pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb8pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb8pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb8pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb8pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb8pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb8pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategy::groundbot9Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb9pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb9pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb9pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb9pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb9pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb9pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb9pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb9pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategy::groundbot10Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb10pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb10pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb10pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb10pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb10pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb10pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb10pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb10pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategy::groundbot11Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb11pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb11pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb11pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb11pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb11pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb11pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb11pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb11pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategy::groundbot12Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb12pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb12pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb12pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb12pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb12pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb12pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb12pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb12pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}

void strategy::groundbot13Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  gb13pose.pose.pose.position.x = msg->pose.pose.position.x;
  gb13pose.pose.pose.position.y = msg->pose.pose.position.y;
  gb13pose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  gb13pose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  gb13pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  gb13pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  gb13pose.twist.twist.linear.x = msg->twist.twist.linear.x;
  gb13pose.twist.twist.linear.y = msg->twist.twist.linear.y;
  return;
}
