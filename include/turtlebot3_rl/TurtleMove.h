#ifndef _TURTLE_MOVE_
#define _TURTLE_MOVE_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <string.h>
#include <math.h>
using namespace std;

class TurtleBot3 {
private:
  // create node handler for communicating with nodes :
  ros::NodeHandle n;

  // laser data :
  ros::Subscriber laser_sub;
  std::vector<float> laser_range;
  std::string laser_topic;
  
  // velocity data :
  ros::Publisher vel_pub;
  geometry_msgs::Twist vel_msg;
  std::string vel_topic;
  
  // odometry data :
  ros::Subscriber odom_sub;
  std::string odom_topic;
  float x_pos; // position along x axis
  float y_pos; // position along y axis
  float z_pos; // position along z axis
  float z_ori; // orientation along z axis

  // private methods :
  // laser data subscriber callback :
  void laser_callback(const sensor_msgs::LaserScan::ConstPtr &laser_msg);
  
  // odometry data subscriber callback :
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);

public:
  // public methods :

  // constructor :
  TurtleBot3();
  
  // move forward for 2 seconds :
  void move();

  // move forward for n_secs seconds :
  void move_forward(int n_secs);
  void move_forward_meters(float meters);
  void move_backwards(int n_secs);
  void turn(string clock, int n_secs);
  void turn_degree(int deg);
  void stop_moving();
  float get_position(int param);
  std::list<float> get_position_full();
  double get_time();
  float get_laser(int index);
  float *get_laser_full();
};
#endif