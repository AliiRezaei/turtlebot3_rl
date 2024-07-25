#include "turtlebot3_rl/TurtleMove.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <string.h>
#include <math.h>
#include <random>
#include <iostream>
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


// int calculate_rewards(int previous_distance, int current_distance){
//   if(current_distance < previous_distance)      {return -5;}
//   else if(current_distance > previous_distance) {return -5;}
//   else                                          {return  1;}
// }


// template <typename inputType> int array_length(inputType arr[]) {
//   int len = *(&arr + 1) - arr;
//   return len;
// }






float eucliden_distance(float P1[], float P2[]) {
  float norm = 0.0;
  for(int i=0; i<2; i++) {
    norm += (P1[i] - P2[i]) * (P1[i] - P2[i]);
  }
  return sqrt(norm);
}



int calculate_rewards(int previous_distance, int current_distance){
  if(current_distance < previous_distance)      {return -5;}
  else if(current_distance > previous_distance) {return -5;}
  else                                          {return  1;}
}








float *calculate_action_values(int reward[], int action_count[]){
  // init action_values :
  float action_values[5];

  /// calculate action_values :
  for(int i=0; i<5; i++){
    if(action_count[i] == 0) {
      action_values[i] = 0.0;
    }
    else {
      action_values[i] = reward[i] / action_count[i];
    }
  }
  float *action_values_ptr = action_values;
  return action_values_ptr;
}




float randUniform(float low, float up){
  std::random_device rd;
  std::mt19937 eng(rd());
  std::uniform_real_distribution<>distr_real(low, up);
  float rand = distr_real(eng);
  return rand;
}

float randInt(int low, int up){
  std::random_device rd;
  std::mt19937 eng(rd());
  std::uniform_int_distribution<>distr(low, up);
  float rand = distr(eng);
  return rand;
}






int main(int argc, char **argv)
{
  ros::init(argc, argv, "FuckingNode");
  TurtleBot3 robot;

  int rewards[] = {0, 0, 0, 0, 0};
  int action_count[] = {0, 0, 0, 0, 0};
  float epsilon = 0.05;

  float previous_distance = robot.get_laser(0);

  while(1){
    float rand = randUniform(0.0, 1.0);
    int action = -1;

    if(rand < epsilon) {
      // select a random acttion :
      action = randInt(0, 4);
    }
    else {
      float *action_values_ptr = calculate_action_values(rewards, action_count);
      // calculate argmax action_values :
      double max_action_values    = - 100000.0;
      int   argmax_action_values =       0;
      for(int i=0; i<5; i++){
        if(*(action_values_ptr+i) > max_action_values){
          max_action_values = *(action_values_ptr+i);
          argmax_action_values = i;
        }
      }
      // select optimal action :
      action = argmax_action_values;
    }

    

    if (action == 0) {
      // do nothing :
      robot.stop_moving();
    }
    else if (action == 1) {
      // move forward :
      robot.move_forward(1);
    }
    else if (action == 2) {
      // move backward :
      robot.move_backwards(1);
    }
    else if (action == 3) {
      // turn right :
      robot.turn("clockwise", 1);
    }
    else if (action == 4) {
      // turn left :
      robot.turn("counterclockwise", 1);
    }
    else {
      ROS_INFO_STREAM("Unrecognized Action ");
      break;
    }

  float current_distance = robot.get_laser(0);
  int reward = calculate_rewards(previous_distance, current_distance);
  rewards[action] = rewards[action] + reward;


  }

  return 0;
}


// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "FuckingNode");
//     TurtleBot3 robot;
//     // TurtleBot3();

//     std::cout << "Fuck!";

//     robot.move_forward(2);

//     return 0;
// }

