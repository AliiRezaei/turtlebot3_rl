#ifndef _TURTLE_MOVE_
#define _TURTLE_MOVE_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <string.h>
#include <math.h>
#include <turtlebot3_rl/Toolbox.h>


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

  // go to the target pose :
  void go_target(float *current_pose, float *target_pose);

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
  // ROS_INFO("Initializing node .................................");
  usleep(1000000);
  ros::spinOnce();
}

void TurtleBot3::laser_callback(const sensor_msgs::LaserScan::ConstPtr &laser_msg) {
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
  
  // turtlebot3 quaternion x_quat:
  x_quat = odom_msg->pose.pose.orientation.x;

  // turtlebot3 quaternion y_quat:
  y_quat = odom_msg->pose.pose.orientation.y;
  
  // turtlebot3 quaternion z_quat:
  z_quat = odom_msg->pose.pose.orientation.z;
  
  // turtlebot3 quaternion y_quat:
  w_quat = odom_msg->pose.pose.orientation.w;

  // quat to rpy :
  Quaternion2RollPitchYaw();

}

void TurtleBot3::Quaternion2RollPitchYaw() {

  this->roll  = atan2(2.0*(y_quat*z_quat + w_quat*x_quat), w_quat*w_quat - x_quat*x_quat - y_quat*y_quat + z_quat*z_quat);
  this->pitch = asin(-2.0*(x_quat*z_quat - w_quat*y_quat));
  this->yaw   = atan2(2.0*(x_quat*y_quat + w_quat*z_quat), w_quat*w_quat + x_quat*x_quat - y_quat*y_quat - z_quat*z_quat);

}
void TurtleBot3::move() {
  // Rate of publishing
  ros::Rate rate(10);

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(2.0); // Timeout of 2 seconds
  while (ros::Time::now() - start_time < timeout) {
    ros::spinOnce();
    vel_msg.linear.x  = 0.2;
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
    // ROS_INFO_STREAM("Moving forward ........... ");
    ros::spinOnce();
    
    // adjust velocity and publish them :
    vel_msg.linear.x = 0.2;
    vel_msg.linear.y = 0.0;
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

// void TurtleBot3::move_forward_meters(float meters) 
// {
//   // ROS_INFO_STREAM("Robot has started from x = " << x_pos);

//   // rate of publishing :
//   ros::Rate rate(10);
//   ros::Time start_time = ros::Time::now();

//   // target position and stop criteria :
//   float x_target = x_pos + meters;
//   double eps = 0.0001;

//   // go forward until reaching target position :
//   while (x_target - x_pos > eps) {
//     // ROS_INFO_STREAM("Moving forward " << meters << " meters");
//     ros::spinOnce();
//     vel_msg.linear.x = 0.2;
//     // vel_msg.linear.x = 0.3 * (x_target - x_pos);
//     vel_msg.linear.y = 0.0;
//     vel_msg.angular.z = 0.0;
//     vel_pub.publish(vel_msg);
//     rate.sleep();
//   }

//   // stop moving :
//   vel_msg.linear.x = 0.0;
//   vel_msg.angular.z = 0.0;
//   vel_pub.publish(vel_msg);
  
//   // print robot moving status :
//   ros::Time end_time = ros::Time::now();
//   // ROS_INFO_STREAM("Robot reached " << x_pos << " after " << end_time - start_time << " seconds");
// }


void TurtleBot3::move_forward_meters(float meters) {
  vel_msg.linear.x  = 0.08;
  vel_msg.linear.y  = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}




void TurtleBot3::move_backward(int time) {
  // Rate of publishing
  ros::Rate rate(10);

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(time);
  while (ros::Time::now() - start_time < timeout) {
    // ROS_INFO_STREAM("Moving backwards ........... ");
    ros::spinOnce();
    vel_msg.linear.x = -0.2;
    vel_msg.linear.y = 0.0;
    vel_msg.angular.z = 0.0;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

// void TurtleBot3::move_backward_meters(float meters) 
// {
//   // ROS_INFO_STREAM("Robot has started from x = " << x_pos);

//   // rate of publishing :
//   ros::Rate rate(10);
//   ros::Time start_time = ros::Time::now();

//   // target position and stop criteria :
//   float x_target = x_pos - meters;
//   double eps = 0.0001;

//   // go forward until reaching target position :
//   while (x_pos - x_target > eps) {
//     // ROS_INFO_STREAM("Moving forward " << meters << " meters");
//     ros::spinOnce();
//     vel_msg.linear.x = -0.2;
//     // vel_msg.linear.x = -0.3 * (x_pos - x_target);
//     vel_msg.linear.y = 0.0;
//     vel_msg.angular.z = 0.0;
//     vel_pub.publish(vel_msg);
//     rate.sleep();
//   }

//   // stop moving :
//   vel_msg.linear.x = 0.0;
//   vel_msg.angular.z = 0.0;
//   vel_pub.publish(vel_msg);
  
//   // print robot moving status :
//   ros::Time end_time = ros::Time::now();
//   // ROS_INFO_STREAM("Robot reached " << x_pos << " after " << end_time - start_time << " seconds");
// }

void TurtleBot3::move_backward_meters(float meters) {
  vel_msg.linear.x  = - 0.08;
  vel_msg.linear.y  =   0.0;
  vel_msg.angular.z =   0.0;
  vel_pub.publish(vel_msg);
}


void TurtleBot3::turn(string clock, int n_secs) {
  ros::Rate rate(10);
  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(n_secs);

  double WZ = 0.0;
  if (clock == "clockwise") {
    // ROS_INFO_STREAM("Turning clockwise..............");
    WZ = -2.5;
  } else if (clock == "counterclockwise") {
    // ROS_INFO_STREAM("Turning counterclockwise ........... ");
    WZ = 2.5;
  }

  while (ros::Time::now() - start_time < timeout) {
    ros::spinOnce();
    vel_msg.linear.x = 0.0;
    vel_msg.angular.z = WZ;
    vel_pub.publish(vel_msg);
    rate.sleep();
  }
  vel_msg.linear.x = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void TurtleBot3::turn_degree(float deg)
{
  // Rate of publishing
  ros::Rate rate(10);

  double theta_target = deg * 3.14159265 / 180.0 + yaw;

  int sign_theta = 0;
  if(deg > 0.0) {sign_theta =  1;}
  if(deg < 0.0) {sign_theta = -1;}

  double eps = 0.0001;
  while(sign_theta * (theta_target - yaw) > eps)
  {
    ros::spinOnce();
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    vel_msg.angular.z = sign_theta * 0.3;
    vel_pub.publish(vel_msg);
    // ROS_INFO_STREAM("Turning " << deg << " degrees");
    rate.sleep(); 
  }
  vel_msg.linear.x = 0.0;
  vel_msg.linear.y = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}


void TurtleBot3::stop_moving() {
  // ROS_INFO_STREAM("Stopping the robot ........... ");
  vel_msg.linear.x = 0.0;
  vel_msg.linear.y = 0.0;
  vel_msg.angular.z = 0.0;
  vel_pub.publish(vel_msg);
}

void TurtleBot3::go_target(float *current_pose, float *target_pose) {


  // extract target_pose :
  float target_x     = *(target_pose + 0);
  float target_y     = *(target_pose + 1);
  float target_theta = *(target_pose + 2);

  // extract current_pose :
  float current_x     = *(current_pose + 0);
  float current_y     = *(current_pose + 1);
  float current_theta = *(current_pose + 2);

  // normalize target_theta :
  if(target_theta > _PI_NUMBER_) {target_theta -= 2.0 * _PI_NUMBER_;}

  // init loop params :
  float distance;   // distance between actual position and target position
  float lambda;     // distance between actual orientation and target orientation 
  float alpha;      // auxiliary var
  float beta;       // auxiliary var
  float v;          // linear  vel 
  float w;          // angular vel
  float v_scal;     // scaled linear  vel 
  float w_scal;     // scaled angular vel

  // gains :
    float K_distannce =   2.0;
    float K_alpha     =  15.0;
    float K_beta      = - 3.0;
    float V_const     =   0.1;

  // reaching precise :
  float eps = 0.0001;

  // reaching status :
  bool robot_in_target = false;

  // main loop :
  while(!robot_in_target) {

    // update distance :
    distance = eucliden_distance(current_pose, target_pose, 2);

    // update lambda :
    lambda = atan2(target_y - current_y, target_x - current_x);

    // update alpha and beta :
    alpha = (lambda -  current_theta + pi) % (2 * _PI_NUMBER_) - _PI_NUMBER_;
    beta  = (target_theta - lambda + pi) % (2 * _PI_NUMBER_) - _PI_NUMBER_;

    // check reaching status :
    if(distance < eps && abs(current_theta - target_theta) < eps) {
      vel_msg.linear.x  = 0.0;
      vel_msg.linear.y  = 0.0;
      vel_msg.angular.z = 0.0;
      vel_pub.publish(vel_msg);
      robot_in_target = true;
    }
    else {
      float v = K_distannce * distance;
      float w = K_alpha * alpha + K_beta * beta;
      float v_scal = v / abs(v) * V_const;
      float w_scal = w / abs(v) * V_const;

      vel_msg.linear.x  = v_scal;
      vel_msg.linear.y  = 0.0;
      vel_msg.angular.z = w_scal;
      vel_pub.publish(vel_msg);
    }

    // update pose :
    *(current_pose + 0) = this->x_pos;
    *(current_pose + 1) = this->y_pos;
    *(current_pose + 2) = this->yaw;
  }
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

float TurtleBot3::get_laser(int index) {
  return laser_range[index];
}

float *TurtleBot3::get_laser_full() {
  float *laser_range_ptr = laser_range.data();
  return laser_range_ptr;
}

#endif