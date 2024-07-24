#include "turtlebot3_rl/TurtleMove.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <string.h>
#include <math.h>
using namespace std;

// TurtleBot3 constructor
TurtleBot3::TurtleBot3() {
  
  // initializing node :
  n = ros::NodeHandle("~");
  
  // laser topic and subscriber :
  laser_topic = "/scan";
  laser_sub = n.subscribe(laser_topic, 10, &TurtleBot3::laser_callback, this);
  
  // velocity command topic and publisher :
  vel_topic = "/cmd_vel";
  vel_pub = n.advertise<geometry_msgs::Twist>(n.resolveName(vel_topic), 1);
  
  // odometry topic and subscriber :
  odom_topic = "/odom";
  odom_sub = n.subscribe(odom_topic, 10, &TurtleBot3::odom_callback, this);
  
  // log init info :
  ROS_INFO("Initializing node .................................");
  usleep(2000000);
  ros::spinOnce();
}

void TurtleBot3::laser_callback(
    const sensor_msgs::LaserScan::ConstPtr &laser_msg) {
  laser_range = laser_msg->ranges;
  // ROS_INFO("Laser value: %f", laser_range);
}
void TurtleBot3::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  
  // turtlebot3 position along x axis :
  x_pos = odom_msg->pose.pose.position.x;
  
  // turtlebot3 position along y axis :
  y_pos = odom_msg->pose.pose.position.y;
  
  // turtlebot3 position along z axis :
  z_pos = odom_msg->pose.pose.position.z;
  
  // turtlebot3 orientation along z axis (theta):
  z_ori = odom_msg->pose.pose.orientation.z;
}
void TurtleBot3::move() {
  // Rate of publishing
  ros::Rate rate(10);

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(2.0); // Timeout of 2 seconds
  while (ros::Time::now() - start_time < timeout) {
    ros::spinOnce();
    vel_msg.linear.x = +0.5;
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void TurtleBot3::move_forward(int time) {
  
  // rate of publishing :
  int freq = 10;
  ros::Rate rate(freq);

  // start time and duration :
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time);

  // go forward for "time" seconds :
  while (ros::Time::now() - start_time < timeout) {
    ROS_INFO_STREAM("Moving forward ........... ");
    ros::spinOnce();
    
    // adjust velocity and publish them :
    vel_msg.linear.x = 0.2;
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);

    // waiting :
    rate.sleep();
  }

  // publish zero velocity command :
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
} // end of move_forward

void TurtleBot3::move_forward_meters(float meters) 
{
  ROS_INFO_STREAM("Robot has started from x = " << x_pos);

  // rate of publishing :
  ros::Rate rate(10);
  ros::Time start_time = ros::Time::now();

  // target position and stop criteria :
  float x_target = x_pos + meters;
  float eps = 0.001;

  // go forward until reaching target position :
  while (x_target - x_pos > eps) {
    ROS_INFO_STREAM("Moving forward " << meters << " meters");
    ros::spinOnce();
    vel_msg.linear.x = 0.2;
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }

  // stop moving :
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
  
  // print robot moving status :
  ros::Time end_time = ros::Time::now();
  ROS_INFO_STREAM("Robot reached " << x_pos << " after " << end_time - start_time << " seconds");
}

void TurtleBot3::move_backwards(int time) {
  // Rate of publishing
  ros::Rate rate(10);

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time);
  while (ros::Time::now() - start_time < timeout) {
    ROS_INFO_STREAM("Moving backwards ........... ");
    ros::spinOnce();
    vel_msg.linear.x = -0.5;
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void TurtleBot3::turn(string clock, int n_secs) {
  ros::Rate rate(10);
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(n_secs);

  double WZ = 0.0;
  if (clock == "clockwise") {
    ROS_INFO_STREAM("Turning clockwise..............");
    WZ = -2.5;
  } else if (clock == "counterclockwise") {
    ROS_INFO_STREAM("Turning counterclockwise ........... ");
    WZ = 2.5;
  }

  while (ros::Time::now() - start_time < timeout) {
    ros::spinOnce();
    cout  << z_ori << endl;
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = WZ;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void TurtleBot3::turn_degree(int deg)
{
  // Rate of publishing
  ros::Rate rate(10);

  float theta_now = z_ori * 3.1415;
  float theta_target = deg * 3.1415 / 180.0 + theta_now;

  int sign_theta = 0;
  if(deg > 0) {sign_theta =  1;}
  if(deg < 0) {sign_theta = -1;}

  float eps = 0.01;
  while(theta_target - z_ori * 3.1415 > eps)
  {
    ros::spinOnce();
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = sign_theta * 1.5;
    vel_pub.publish(vel_msg);
    ROS_INFO_STREAM("Turning " << deg << " degrees");
    cout << z_ori << endl;
    rate.sleep(); 
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void TurtleBot3::stop_moving() {
  ROS_INFO_STREAM("Stopping the robot ........... ");
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

float TurtleBot3::get_position(int param) {
  if (param == 1) {
    return this->x_pos;
  } else if (param == 2) {
    return this->y_pos;
  } else if (param == 3) {
    return this->z_pos;
  }
  return 0;
}

list<float> TurtleBot3::get_position_full() {
  list<float> coordinates({this->x_pos, this->y_pos, this->z_pos});
  return coordinates;
}

double TurtleBot3::get_time() {
  double secs = ros::Time::now().toSec();
  return secs;
}

float TurtleBot3::get_laser(int index) { return laser_range[index]; }

float *TurtleBot3::get_laser_full() {
  float *laser_range_pointer = laser_range.data();
  return laser_range_pointer;
}

