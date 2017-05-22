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
  	distance_bots.push_back(temp.pose.pose.position.y);
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

    fly_quad.navigate_quad(bots[z]);

		rotate(PI,publish_name,bots[z]);
    	z++;
	}
}


void strategy::ComputeDistance()                                     //computes distance of every bot from the green line
{
	
  for(int i=4;i<14;i++)
  {
  	int flag=1;
  	if((!bots_removed.empty()))
  	{
  		for(int j=0; j<bots_removed.size(); j++)
  		{
  			if(bots_removed[j]==i)
  				flag=0;
  		}
  		if(flag==1)
  		{
	  		nav_msgs:: Odometry temp;
			ros::Rate loop_rate(10);
			ros::spinOnce();
			loop_rate.sleep();
	   	    retrieve_pose(i, &temp);
			double y = 10 - temp.pose.pose.position.y;
			ClosestBot.insert(make_pair(y,i));
  		}
  		
  	}
  	else
  	{
  		nav_msgs:: Odometry temp;
		ros::Rate loop_rate(10);
		ros::spinOnce();
		loop_rate.sleep();
   	    retrieve_pose(i, &temp);
		double y = 10 - temp.pose.pose.position.y;
		ClosestBot.insert(make_pair(y,i));
  	}
    
  }
}


void strategy::FindBotsInsideCircle()                               //to find the bot inside the 5m circle
{
	int centerBotID;
	p temp = *ClosestBot.begin();
	centerBotID = temp.second;

  ROS_INFO("center bot id:::::%d\n", centerBotID);
	
  nav_msgs::Odometry center;
  nav_msgs::Odometry inside;

	int count=4;
	while(count<14)
  {
		if(count!=centerBotID)
    {
			ros::Rate loop_rate(10);
			ros::spinOnce();
			loop_rate.sleep();
      retrieve_pose(centerBotID, &center);
      retrieve_pose(count, &inside);
     }
  	double dist = pow((inside.pose.pose.position.x - center.pose.pose.position.x),2) + pow((inside.pose.pose.position.y - center.pose.pose.position.y),2);
  	if((dist - 25)<=0 && count!=centerBotID)
    {
  		BotsInsideCircle.push_back(count);
      ROS_INFO("bot inside circle::::::%d\n", count);
    }
  	count++;
  }
}


void strategy::FirstOperation()                                          //to decide the first operation of the center bot
{
	double yaw=0, pitch=0, roll=0;
	int centerBotID;
	p temp = *ClosestBot.begin();
	centerBotID = temp.second;

	char publish_name[40];
  sprintf(publish_name, "robot%d/cmd_vel", centerBotID);
	fly_quad.navigate_quad(centerBotID);

  nav_msgs::Odometry center;
  qt centerq;
  ros::Rate loop_rate(10);
  ros::spinOnce();
  loop_rate.sleep();

  retrieve_pose(centerBotID, &center);

  centerq.x = center.pose.pose.orientation.x;
  centerq.y = center.pose.pose.orientation.y;
  centerq.z = center.pose.pose.orientation.z;
  centerq.w = center.pose.pose.orientation.w;

	double theta1 = atan((10 - center.pose.pose.position.y)/(10 - center.pose.pose.position.x));
	double theta2 = PI - atan((10 - center.pose.pose.position.y)/(-10 - center.pose.pose.position.x));

	GetEulerAngles(centerq, &yaw, &pitch, &roll);

	if(yaw>=theta1 && yaw<=theta2)
		ROS_INFO("Condition 0::::::do nothing\n");

	else if((yaw>=angle(theta1+PI) && yaw<=angle(theta2+PI) ) || (angle(theta1+PI)*angle(theta2+PI)<0 && (yaw>=angle(theta1+PI) || yaw<=angle(theta2+PI)) ) )
	{
		ROS_INFO("Condition 1::::::180 degree turn %f\n",yaw);
    rotate(PI,publish_name,centerBotID);
	}
	else if((yaw>theta2 && yaw<angle(theta2+PI/4)) || ( theta2*angle(theta2+PI/4)<0 && (yaw>theta2 || yaw<angle(theta2+PI/4))))
	{
		ROS_INFO("Condition 2::::::45 degree turn \n");
    rotate((PI/4),publish_name,centerBotID);
	}
	else if(yaw>=angle(theta2 + PI/4) && yaw<=angle(theta2 + PI/2) || ( angle(theta2+PI/4)*angle(theta2+PI/2)<0 && (yaw>angle(theta1+PI/4) || yaw<angle(theta2+PI/2))))
	{
		ROS_INFO("Condition 3::::::90 degree turn, tap twice\n");
    rotate((PI/2),publish_name,centerBotID);
	}
	else if(yaw>=angle(theta2+PI) && yaw<=angle(theta2+PI+PI/4) || (angle(theta2+PI)*angle(theta2+PI+PI/4) && (yaw>=angle(theta2+PI) && yaw<=angle(theta2+PI+PI/4))))
	{
		ROS_INFO("Condition 4::::::180 then 45 degree turn\n");
    rotate((5*PI/4),publish_name,centerBotID);
	}
}

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
    //ROS_INFO("in while yaw:::%f\n", Yaw);
    if(fabs(angle(Yaw) - angle(yaw_i+relative_angle)) <= 0.1)
  	 break;

}

ROS_INFO("after %f\n",Yaw);
 vel_msg.angular.z = 0;
 publi.publish(vel_msg);
}




int strategy::IsOutsideWhite()
{
	int centerBotID;
	p temp = *ClosestBot.begin();
	centerBotID = temp.second;
	nav_msgs::Odometry tempo;
	ros::Rate loop_rate(10);
	ros::spinOnce();
	loop_rate.sleep();
	retrieve_pose(centerBotID, &tempo);
	if(tempo.pose.pose.position.y<=MIN_DIST)
	{
		return 1;
	}
	else
	{
		bots_removed.push_back(centerBotID);
		return 0;
	}
}



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



float strategy::dist_whitel(int id){

  ros::Rate loop_rate(100);
  nav_msgs::Odometry temp1;

  ros::spinOnce();
  loop_rate.sleep();
  retrieve_pose(id, &temp1);
  double posX=temp1.pose.pose.position.x;
  double posY=temp1.pose.pose.position.y;

  qt q;
  q.x = temp1.pose.pose.orientation.x;
  q.y = temp1.pose.pose.orientation.y;
  q.z = temp1.pose.pose.orientation.z;
  q.w = temp1.pose.pose.orientation.w;

  double yaw,pitch,roll;
  GetEulerAngles(q,&yaw,&pitch,&roll);
  float orient=yaw;

  if(orient>=0 && orient<PI/2){
      return 10-posY;
  }

  if(orient>=PI/2 && orient>PI){
      if(10-posY<10+posX)
        return 10-posY;
      else
        return 10+posX;
  }

  if(orient<0 && orient>-PI/2){
      return 10+posY;
  }

  if(orient>-PI && orient<-PI/2){
  	if(10+posY<10+posX)
      return 10+posY;
  	else
  		return 10+posX;
  }
}

float strategy::angle(float ang){

  if(ang>=-PI && ang<=PI)
    return ang;
  else if(ang>PI)
    return angle(ang-2*PI);
  else if(ang<-PI)
    return angle(ang+2*PI);

}


void strategy::action(int bot_no){

  float theta1,theta2,orient;
  char publish_name[40];
  sprintf(publish_name, "robot%d/cmd_vel", bot_no);


  nav_msgs::Odometry temp1;
  nav_msgs::Odometry temp2;
  ros::Rate loop_rate(100);
  ros::spinOnce();
  loop_rate.sleep();
  retrieve_pose(bot_no, &temp1);
  double posX=temp1.pose.pose.position.x;
  double posY=temp1.pose.pose.position.y;
  double centerX=temp2.pose.pose.position.x;
  double centerY=temp2.pose.pose.position.y;

  qt q;
  q.x = temp1.pose.pose.orientation.x;
  q.y = temp1.pose.pose.orientation.y;
  q.z = temp1.pose.pose.orientation.z;
  q.w = temp1.pose.pose.orientation.w;

  theta1=angle(atan((centerX-posX)/(centerY-posY))-PI/4);
  theta2=angle(theta1 + PI/2);

  double yaw,pitch,roll;
  GetEulerAngles(q,&yaw,&pitch,&roll);
  orient=yaw;

  if( (orient>=theta2 && orient<=angle(theta2+PI/4)) || ( theta2*angle(theta2-PI/4)<0 && (orient>theta2 || orient<angle(theta2+PI/4) ) ) ){
    //one 45 degree rotation clockwise
  	//yaw=angle(yaw-PI/4);
  	ROS_INFO(":::::::::::condition 1\n");
    rotate(PI/4,publish_name,bot_no);
  }
  else if( (orient>=angle(theta1+PI) && orient<=angle(theta2+PI) ) || (angle(theta1+PI)*angle(theta2+PI)<0 && (orient>=angle(theta1+PI) || orient<=angle(theta2+PI)) ) ){
    //turn 180 degree
    //yaw=angle(yaw-PI);
    ROS_INFO("::::::::::::::condition 2\n");
    rotate(PI,publish_name,bot_no);
  }
  else if( (orient<=theta1 && orient>=angle(theta1-PI/4)) || ( theta1*angle(theta1-PI/4)<0 && (orient>=angle(theta1-PI/4) || orient<=theta1 ) ) ){
    //one 180 degree turn and then two 45 degree clockwise rotation
    //yaw=angle(yaw+3*PI/2);
    ROS_INFO("::::::::::::::::condition 3\n");
    rotate(3*PI/2,publish_name,bot_no);
  }
  else if( (orient>=angle(theta2+PI/4) && orient<=angle(theta2+PI/2)) || ( angle(theta2+PI/4)*angle(theta2+PI/2)<=0 && (orient>angle(theta1+PI/4) || orient<angle(theta2+PI/2) ) )){
    //two 45 degree anticlockwise rotations
    //yaw=angle(yaw+PI/2);
    ROS_INFO(":::::::::::::::condition 4\n");
    rotate(PI/2,publish_name,bot_no);
  }
  else if( (orient<angle(theta1-PI/4) && orient>=angle(theta1-PI/2)) || ( angle(theta1-PI/2)*angle(theta1-PI/4)<0 && (orient>angle(theta1-PI/2) || orient<angle(theta1-PI/4) ) )){
    //one 180 degree turn and one 45 degree clockwise turn
    //yaw=angle(yaw-5*PI/4);
    ROS_INFO("::::::::::::::::condition 5\n");
    rotate(5*PI/4,publish_name,bot_no);
  }

}


void strategy::t_plan(){

  int centerBotID;
  p temp = *ClosestBot.begin();
  centerBotID = temp.second;

  int size=BotsInsideCircle.size();

  nav_msgs::Odometry temp1;
  ros::Rate loop_rate(100);
  ros::spinOnce();
  loop_rate.sleep();
  retrieve_pose(centerBotID, &temp1);
  double centerX=temp1.pose.pose.position.x;
  double centerY=temp1.pose.pose.position.y;

  float dis,min;
  int bot_no=BotsInsideCircle[0];
  char topic_name2[40];

  for(int i=0;i<size;i++)
  {
    dis=dist_whitel(BotsInsideCircle[i]);
    if(i==0)
      min=dis;
    else if(dis<min){
      min=dis;
      bot_no=BotsInsideCircle[i];
  }

  }
  ROS_INFO("bot inside the circle to be considered %d\n",bot_no);

  action(bot_no);
}


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
