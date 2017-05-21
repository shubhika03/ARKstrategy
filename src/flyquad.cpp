#include <ros/ros.h>
#include <ark_llp/go2goal.h>
#include <time.h>
#include <math.h>
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"
#include "nav_msgs/Odometry.h"
#include "../include/strategy/flyquad.h"

#define step 0.1              // step for changing altitude gradually
#define Eps 0.4            // range for error
#define Default 2.5          // Default height for the quad
#define Delay 5              // time duration for which it is idle in front of the ground bot
#define GBHeight 0.091948

bool down = false;


int tap_n_turn::navigate_quad(int ID)
{

  ros::Subscriber MAVpose_sub = n.subscribe("/ground_truth/state", 100, &tap_n_turn::MAVCallback,this); // subscriber to get MAV position
  ros::Subscriber gbpose_sub_4 = n.subscribe("/robot4/odom", 100, &tap_n_turn::groundbot4Callback,this); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_5 = n.subscribe("/robot5/odom", 100, &tap_n_turn::groundbot5Callback,this); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_6 = n.subscribe("/robot6/odom", 100, &tap_n_turn::groundbot6Callback,this); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_7 = n.subscribe("/robot7/odom", 100, &tap_n_turn::groundbot7Callback,this); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_8 = n.subscribe("/robot8/odom", 100, &tap_n_turn::groundbot8Callback, this); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_9 = n.subscribe("/robot9/odom", 100, &tap_n_turn::groundbot9Callback, this); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_10 = n.subscribe("/robot10/odom", 100, &tap_n_turn::groundbot10Callback,this); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_11 = n.subscribe("/robot11/odom", 100, &tap_n_turn::groundbot11Callback, this); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_12 = n.subscribe("/robot12/odom", 100, &tap_n_turn::groundbot12Callback,this); // subscriber to get ground bot position
  ros::Subscriber gbpose_sub_13 = n.subscribe("/robot13/odom", 100, &tap_n_turn::groundbot13Callback,this); // subscriber to get ground bot position

  ros::Rate loop_rate(10);


  while(ros::ok())
  {

    theta = GetTheta(ID);

    MAVdest.pose.pose.position.x = gbpose.pose.pose.position.x + (t0)*(gbpose.twist.twist.linear.x)*(cos(theta));
    MAVdest.pose.pose.position.y = gbpose.pose.pose.position.y +  (t0)*(gbpose.twist.twist.linear.x)*(sin(theta));

    ErrorLin = GetErrorLin(MAVdest,MAVpose);                       //error between expected and actual position of quad

    if(ErrorLin > Eps && down == false)
    {
      follow();
      ROS_INFO("following\n");
    }

    else if(ErrorLin < Eps || down == true)
    {
      if(count!=0)
      {
        ErrorLin2 = GetErrorLin2(gbpose,MAVpose);
        if(ErrorLin2 <= fabs((t0)*(gbpose.twist.twist.linear.x)))
        {
        	descent();
        	 ROS_INFO("descend\n");
    	 }
        else
        	down = false;
        if (MAVpose.pose.pose.position.z<=GBHeight)
        {
          ascent();
          ROS_INFO("ascend:::reached\n");
          return 0;
        }
      }
      count ++;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return (0);
}

void tap_n_turn::groundbot4Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void tap_n_turn::groundbot5Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void tap_n_turn::groundbot6Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void tap_n_turn::groundbot7Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void tap_n_turn::groundbot8Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void tap_n_turn::groundbot9Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void tap_n_turn::groundbot10Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void tap_n_turn::groundbot11Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void tap_n_turn::groundbot12Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void tap_n_turn::groundbot13Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

void tap_n_turn::MAVCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //if(check==0)
  {
  MAVpose.pose.pose.position.x = msg->pose.pose.position.x;
  MAVpose.pose.pose.position.y = msg->pose.pose.position.y;
  MAVpose.pose.pose.position.z = msg->pose.pose.position.z;
  MAVpose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
  MAVpose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
  MAVpose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
  MAVpose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  MAVpose.twist.twist.linear.x = msg->twist.twist.linear.x;
  MAVpose.twist.twist.linear.y = msg->twist.twist.linear.y;
  MAVpose.twist.twist.linear.z = msg->twist.twist.linear.z;
  MAVpose.twist.twist.angular.z = msg->twist.twist.angular.z;
  }
  return;
}



float tap_n_turn::GetErrorLin(const nav_msgs::Odometry MAVdest ,const nav_msgs::Odometry MAVpose)
{
  float El;
  El = sqrt(pow((MAVdest.pose.pose.position.x - MAVpose.pose.pose.position.x),2) + pow((MAVdest.pose.pose.position.y - MAVpose.pose.pose.position.y),2));
  return(El);
}

float tap_n_turn::GetErrorLin2(const nav_msgs::Odometry gbpose ,const nav_msgs::Odometry MAVpose)
{
  float El;
  El = sqrt(pow((gbpose.pose.pose.position.x - MAVpose.pose.pose.position.x),2) + pow((gbpose.pose.pose.position.y - MAVpose.pose.pose.position.y),2));
  return(El);
}

void tap_n_turn::GetEulerAngles(Quaternionm q, double* yaw, double* pitch, double* roll)
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

   void tap_n_turn::follow()
   {
     destination.set_dest((MAVdest.pose.pose.position.y)*(-1),MAVdest.pose.pose.position.x,Default,0);
     ROS_INFO("%f \t %f \t  ",ErrorLin, MAVpose.pose.pose.position.z );

   }

   void tap_n_turn::descent()
   {
    MAVdest.pose.pose.position.z =-30;                                   //descent of MAV
    destination.set_dest((MAVpose.pose.pose.position.y)*(-1),MAVpose.pose.pose.position.x,MAVdest.pose.pose.position.z,0);
    ROS_INFO("%f \t %f \t  ",ErrorLin, MAVpose.pose.pose.position.z );
    down = true;
   }

  void tap_n_turn::ascent()
  {
    destination.set_dest((MAVdest.pose.pose.position.y)*(-1),MAVdest.pose.pose.position.x,Default,0);
     ROS_INFO("%f \t %f \t  ",ErrorLin, MAVpose.pose.pose.position.z );
     down = false;
  }

  double tap_n_turn::GetTheta(int ID)
  {
    Quaternionm myq;

    if(ID == 4)
    {
     myq.x = gb4pose.pose.pose.orientation.x;
     myq.y = gb4pose.pose.pose.orientation.y;
     myq.z = gb4pose.pose.pose.orientation.z;
     myq.w = gb4pose.pose.pose.orientation.w;

     gbpose.pose.pose.position.x = gb4pose.pose.pose.position.x;
     gbpose.pose.pose.position.y = gb4pose.pose.pose.position.y;
     gbpose.pose.pose.position.z = gb4pose.pose.pose.position.z;
     gbpose.twist.twist.linear.x = gb4pose.twist.twist.linear.x;
     gbpose.twist.twist.linear.y = gb4pose.twist.twist.linear.y;
     gbpose.twist.twist.linear.z = gb4pose.twist.twist.linear.z;
    }

    else if (ID == 5)
    {
      myq.x = gb5pose.pose.pose.orientation.x;
      myq.y = gb5pose.pose.pose.orientation.y;
      myq.z = gb5pose.pose.pose.orientation.z;
      myq.w = gb5pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb5pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb5pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb5pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb5pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb5pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb5pose.twist.twist.linear.z;
    }

    else if (ID == 6)
    {
      myq.x = gb6pose.pose.pose.orientation.x;
      myq.y = gb6pose.pose.pose.orientation.y;
      myq.z = gb6pose.pose.pose.orientation.z;
      myq.w = gb6pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb6pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb6pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb6pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb6pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb6pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb6pose.twist.twist.linear.z;
    }

    else if (ID == 7)
    {
      myq.x = gb7pose.pose.pose.orientation.x;
      myq.y = gb7pose.pose.pose.orientation.y;
      myq.z = gb7pose.pose.pose.orientation.z;
      myq.w = gb7pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb7pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb7pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb7pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb7pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb7pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb7pose.twist.twist.linear.z;
    }

    else if (ID == 8)
    {
      myq.x = gb8pose.pose.pose.orientation.x;
      myq.y = gb8pose.pose.pose.orientation.y;
      myq.z = gb8pose.pose.pose.orientation.z;
      myq.w = gb8pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb8pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb8pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb8pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb8pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb8pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb8pose.twist.twist.linear.z;
    }

    else if (ID == 9)
    {
      myq.x = gb9pose.pose.pose.orientation.x;
      myq.y = gb9pose.pose.pose.orientation.y;
      myq.z = gb9pose.pose.pose.orientation.z;
      myq.w = gb9pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb9pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb9pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb9pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb9pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb9pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb9pose.twist.twist.linear.z;
    }

    else if (ID == 10)
    {
      myq.x = gb10pose.pose.pose.orientation.x;
      myq.y = gb10pose.pose.pose.orientation.y;
      myq.z = gb10pose.pose.pose.orientation.z;
      myq.w = gb10pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb10pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb10pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb10pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb10pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb10pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb10pose.twist.twist.linear.z;
    }

    else if (ID == 11)
    {
      myq.x = gb11pose.pose.pose.orientation.x;
      myq.y = gb11pose.pose.pose.orientation.y;
      myq.z = gb11pose.pose.pose.orientation.z;
      myq.w = gb11pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb11pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb11pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb11pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb11pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb11pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb11pose.twist.twist.linear.z;
    }

    else if (ID == 12)
    {
      myq.x = gb12pose.pose.pose.orientation.x;
      myq.y = gb12pose.pose.pose.orientation.y;
      myq.z = gb12pose.pose.pose.orientation.z;
      myq.w = gb12pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb12pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb12pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb12pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb12pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb12pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb12pose.twist.twist.linear.z;
    }

    else if (ID == 13)
    {
      myq.x = gb13pose.pose.pose.orientation.x;
      myq.y = gb13pose.pose.pose.orientation.y;
      myq.z = gb13pose.pose.pose.orientation.z;
      myq.w = gb13pose.pose.pose.orientation.w;

      gbpose.pose.pose.position.x = gb13pose.pose.pose.position.x;
      gbpose.pose.pose.position.y = gb13pose.pose.pose.position.y;
      gbpose.pose.pose.position.z = gb13pose.pose.pose.position.z;
      gbpose.twist.twist.linear.x = gb13pose.twist.twist.linear.x;
      gbpose.twist.twist.linear.y = gb13pose.twist.twist.linear.y;
      gbpose.twist.twist.linear.z = gb13pose.twist.twist.linear.z;
    }

    GetEulerAngles(myq, &yaw, &pitch, &roll);

    return(yaw);

  }
