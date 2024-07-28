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

  // position data :
  float x_pos;  // position along x axis
  float y_pos;  // position along y axis
  float z_pos;  // position along z axis
  
  // quaternion data :
  float x_quat; // turtlebot3 quaternion q.x
  float y_quat; // turtlebot3 quaternion q.y
  float z_quat; // turtlebot3 quaternion q.z
  float w_quat; // turtlebot3 quaternion q.y

  // angles data :
  float roll;   // roll  angle
  float pitch;  // pitch angle
  float yaw;    // yaw   angle
  
  // laser data subscriber callback :
  void laser_callback(const sensor_msgs::LaserScan::ConstPtr &laser_msg);
  
  // odometry data subscriber callback :
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);

public:

  // constructor :
  TurtleBot3();
  
  // move forward for 2 seconds :
  void move();

  // extract roll, pitch, yaw from quaternion :
  void Quaternion2RollPitchYaw();

  // move forward for n_secs seconds :
  void move_forward(int n_secs);

  // move forward for arbit meters :
  void move_forward_meters(float meters);

  // move backward for n_secs seconds :
  void move_backward(int n_secs);

  // move backward for arbit meters :
  void move_backward_meters(float meters);

  // turn "cw" or "ccw" for n_secs seconds :
  void turn(string clock, int n_secs);

  // turn arbit degrees :
  void turn_degree(float deg);

  // stop moving :
  void stop_moving();

  // get x, y, z (param should be x or y or z): 
  float get_position(int param);

  // get any x, y, z :
  std::list<float> get_position_full();

  // calculate time :
  double get_time();

  // get laser info in one direction (1 to 720) :
  float get_laser(int index);

  // get laser info in all directions :
  float *get_laser_full();
};
#endif