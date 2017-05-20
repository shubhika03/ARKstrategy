#ifndef VERTICALTAP10_H
#define VERTICALTAP10_H

#include <ros/ros.h>
#include <ark_llp/go2goal.h>
#include <time.h>
#include <math.h>
#include "ardrone_autonomy/Navdata.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"
#include "nav_msgs/Odometry.h"

class tap_n_turn
{

  private:
  float t0;                                                       //descent time
  double theta;                                                   //orientation of gb wrt X-axis
  float ErrorLin;                                                 //Get linear error
  nav_msgs::Odometry gbpose;
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
  nav_msgs::Odometry MAVpose;                                    //position of quad
  nav_msgs::Odometry MAVdest;
  ros::NodeHandle n;                                 //quad destination
  int flag, check, count;
  //int ID;
  double yaw, pitch, roll;
  struct Quaternionm
  {
      double w, x, y, z;
  };

  public:
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
    void MAVCallback(const nav_msgs::Odometry::ConstPtr& msg);
    float GetErrorLin(const nav_msgs::Odometry MAVdest ,const nav_msgs::Odometry MAVpose);
    void GetEulerAngles(Quaternionm q, double* yaw, double* pitch, double* roll);
    double GetTheta(int ID);
    void follow();
    void descent();
    void ascent();
    int navigate_quad(int ID);
    tap_n_turn()
    {
      flag = 0;
      check = 0;
      count = 0;
      t0 = 4;
    }
    Go2Goal destination;
};

#endif
