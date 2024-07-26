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
  // ROS_INFO("Initializing node .................................");
  usleep(2000000);
  ros::spinOnce();
}

void TurtleBot3::laser_callback(
    const sensor_msgs::LaserScan::ConstPtr &laser_msg) {
  laser_range = laser_msg->ranges;
  // // ROS_INFO("Laser value: %f", laser_range);
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

  Quaternion2RollPitchYaw();

}

void TurtleBot3::Quaternion2RollPitchYaw() {

  roll   = atan2(2.0*(y_quat*z_quat + w_quat*x_quat), w_quat*w_quat - x_quat*x_quat - y_quat*y_quat + z_quat*z_quat);
  pitch = asin(-2.0*(x_quat*z_quat - w_quat*y_quat));
  yaw  = atan2(2.0*(x_quat*y_quat + w_quat*z_quat), w_quat*w_quat + x_quat*x_quat - y_quat*y_quat - z_quat*z_quat);

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

void TurtleBot3::move_forward_meters(float meters) 
{
  // ROS_INFO_STREAM("Robot has started from x = " << x_pos);

  // rate of publishing :
  ros::Rate rate(10);
  ros::Time start_time = ros::Time::now();

  // target position and stop criteria :
  float x_target = x_pos + meters;
  double eps = 0.0001;

  // go forward until reaching target position :
  while (x_target - x_pos > eps) {
    // ROS_INFO_STREAM("Moving forward " << meters << " meters");
    ros::spinOnce();
    vel_msg.linear.x = 0.2;
    // vel_msg.linear.x = 0.3 * (x_target - x_pos);
    vel_msg.linear.y = 0.0;
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
  // ROS_INFO_STREAM("Robot reached " << x_pos << " after " << end_time - start_time << " seconds");
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

void TurtleBot3::move_backward_meters(float meters) 
{
  // ROS_INFO_STREAM("Robot has started from x = " << x_pos);

  // rate of publishing :
  ros::Rate rate(10);
  ros::Time start_time = ros::Time::now();

  // target position and stop criteria :
  float x_target = x_pos - meters;
  double eps = 0.0001;

  // go forward until reaching target position :
  while (x_pos - x_target > eps) {
    // ROS_INFO_STREAM("Moving forward " << meters << " meters");
    ros::spinOnce();
    vel_msg.linear.x = -0.2;
    // vel_msg.linear.x = -0.3 * (x_pos - x_target);
    vel_msg.linear.y = 0.0;
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
  // ROS_INFO_STREAM("Robot reached " << x_pos << " after " << end_time - start_time << " seconds");
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
  float *laser_range_ptr = laser_range.data();
  return laser_range_ptr;
}






// template <typename InputType, size_t N> size_t array_length(InputType (&arr)[N]) {
//     return N;
// }



template<typename InputType> float *linspace(InputType a, InputType b, int n) {

    float step_size = (b - a) / (n - 1.0);
    float* sequence = new float[n];

    for(int i = 0; i < n; i++) {
        sequence[i] = a + i * step_size;
    }
    return sequence;
}


double sum_array(double arr[]) {

  double sum = 0.0;
  for(int i=0; i<5; i++) {
    sum += arr[i];
  }
  return sum;
}


float eucliden_distance(float P1[], float P2[]) {
  // size_t lenP1 = array_length(P1);
  // cout << lenP1 << endl;
  float norm = 0.0;
  for(int i=0; i<2; i++) {
    norm += (P1[i] - P2[i]) * (P1[i] - P2[i]);
  }
  return sqrt(norm);
}



float calculate_rewards(float current_position[], float previous_position[], float goal_position[], float collision, int action){
  float reward = 0.0;
  if(eucliden_distance(current_position, goal_position) <= 0.05 && action == 0) {
    reward += 50.0;
  }
  else if(eucliden_distance(current_position, goal_position) <= 0.01 ) {
    reward += 10.0;
  }
  else {
    float previous_distance = eucliden_distance(previous_position, goal_position);
    float current_distance  = eucliden_distance(current_position, goal_position);
    if(current_distance < previous_distance) {
      // reward += 5.0;
      reward += current_distance;
    }
    else {
      // reward -= 5.0;
      reward -= current_distance;
    }

    // if(collision < 0.05) {
    //   reward -= 10.0;
    // }

    // reward -= 1.0;

    if((action == 3)||(action == 4)) {
      reward -= 20.0;
    }
  }

  cout << "current position (x,y) = (" << current_position[0] << "," << current_position[1] << ")" << " , Distance = " << eucliden_distance(current_position, goal_position);
  return reward;
}



// int calculate_rewards(int previous_distance, int current_distance){
//   if(current_distance < previous_distance)      {return -5;}
//   else if(current_distance > previous_distance) {return -5;}
//   else                                          {return  1;}
// }




float *calculate_action_values(double reward[], double action_count[]){
  // init action_values :
  // size_t n = array_length(reward);
  // float a[5];
  // float *action_values = a;
  float *action_values = new float[5];

  /// calculate action_values :
  for(int i=0; i<5; i++){
    if(action_count[i] == 0.0) {
      action_values[i] = 0.0;
    }
    else {
      action_values[i] = reward[i] / action_count[i];
      // double sum_reward = sum_array(reward);
      // action_values[i] = sum_reward / action_count[i];
    }
    cout << "Action Value " << i << " = " << *(action_values + i) << endl;
  }
  cout << endl;
  return action_values;
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



// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "FuckingNode");
//   TurtleBot3 robot;

//   // robot.move_forward_meters(1.0);
//   // robot.move_backwards_meters(1.0);
//   robot.turn_degree(80.0);
  
  
//   return 0;
// }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "FuckingNode");
  TurtleBot3 robot;
  ros::Rate rate(1);

  double rewards[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double action_count[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  float epsilon = 0.05;
  int randomStatus = 0;

  // float previous_distance = robot.get_laser(0);
  float previous_position[2];
  previous_position[0] = robot.get_position(1);
  previous_position[1] = robot.get_position(2);

  int episodCounter = 0;

  while(1){
    float rand = randUniform(0.0, 1.0);
    int action;

    if(rand < epsilon) {
      // select a random acttion :
      action = randInt(0, 4);
      randomStatus = 1;
    }
    else {
      randomStatus = 0;
      float *action_values_ptr = calculate_action_values(rewards, action_count);
      // calculate argmax action_values :
      double max_action_values    = - 100000.0;
      int   argmax_action_values =       0;
      for(int i=0; i<5; i++){


        *(action_values_ptr + i) = *(action_values_ptr + i) + rewards[i];


        if(*(action_values_ptr+i) > max_action_values){
          max_action_values = *(action_values_ptr+i);
          argmax_action_values = i;
        }
      }
      // select optimal action :
      // cout << max_action_values << endl;
      action = argmax_action_values;
    }

    
    

    if (action == 0) {
      // do nothing :
      robot.stop_moving();
    }
    else if (action == 1) {
      // move forward :
      robot.move_forward_meters(0.5);
    }
    else if (action == 2) {
      // move backward :
      robot.move_backward_meters(0.5);
    }
    else if (action == 3) {
      // turn right :
      // robot.turn_degree(90.0);
      robot.stop_moving();
    }
    else if (action == 4) {
      // turn left :
      // robot.turn_degree(-90.0);
      robot.stop_moving();
    }
    else {
      // ROS_INFO_STREAM("Unrecognized Action ");
      break;
      }

    float *collision = robot.get_laser_full();
    int maxDegrees = 360;
    double nearest_obstacle_distance = 100000.0;
    for(int i=0; i<maxDegrees; i+=5) {
      if(*(collision + i) < nearest_obstacle_distance) {
        nearest_obstacle_distance = *(collision + i);
      }
    }

    float current_position[2];
    current_position[0] = robot.get_position(1);
    current_position[1] = robot.get_position(2);

    float goal_position[] = {-1.0, 0.0};

    float reward = calculate_rewards(current_position, previous_position, goal_position, nearest_obstacle_distance, action);
    float gamma = 1.0;
    rewards[action] = gamma * rewards[action] + reward;
    // rewards[action] = rewards[action] + reward;
    action_count[action] = action_count[action] + 1.0;

    // cout << "reward = " << reward << endl;


    // for(int i=0; i<maxDegrees; i+=1) {
    //   cout << "i = " << i << ", collision = " << *(collision + i) << endl;
    // }

    // cout << reward << endl;
    // delete[] action_values_ptr;

    rate.sleep();

    // cout << endl;
    // for(int i=0; i<5; i++){
    //   // cout << i << " th elemnt of action values = " << *(action_values_ptr+i) << " , count = " << action_count[i] << " , reward = " << rewards[i] << endl;
    //   cout << i << " th elemnt : " << " , count = " << action_count[i] << " , reward = " << rewards[i] << endl;
    //   cout << "gift reward = " << reward << "selected action = " << action << "Random? " << randomStatus << endl;
    // }





    cout << endl;
    cout << episodCounter++ << " , gift reward = " << reward << " , selected action = " << action << " , Random? " << randomStatus << endl;


  }
  return 0;
} 
