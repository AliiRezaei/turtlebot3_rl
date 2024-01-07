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
  // Communicate with nodes
  ros::NodeHandle n;
  // Laser data
  ros::Subscriber laser_sub;
  std::vector<float> laser_range;
  std::string laser_topic;
  // Velocity data
  ros::Publisher vel_pub;
  geometry_msgs::Twist vel_msg;
  std::string vel_topic;
  // Odometry data
  ros::Subscriber odom_sub;
  std::string odom_topic;
  float x_pos;
  float y_pos;
  float z_pos;
  float z_ori;

  void laser_callback(const sensor_msgs::LaserScan::ConstPtr &laser_msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);

public:
  TurtleBot3();
  void move();
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