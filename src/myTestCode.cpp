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


template <typename InputType, size_t N>
size_t array_length(InputType (&arr)[N]) {
    return N;
}

class maxFind {
public:
    double maxVal;
    int    maxIdx;
    template <typename InputType> void maxFunc(InputType arr[], size_t len);
};

template <typename InputType> void maxFind::maxFunc(InputType arr[], size_t len) {
    double max_value = -100000.0;
    int    max_index = 0;  
    for(size_t i = 0; i < len; i++){
      if(arr[i] > max_value){
        max_value = arr[i];
        max_index = i;
      }
    }
    this->maxVal = max_value;
    this->maxIdx = max_index;
}

class minFind {
public:
    double minVal;
    int    minIdx;
    template <typename InputType> void minFunc(InputType arr[], size_t len);
};

template <typename InputType> void minFind::minFunc(InputType arr[], size_t len) {
    double min_value = 100000.0;
    int    min_index = 0;  
    for(size_t i = 0; i < len; i++){
      if(arr[i] < min_value){
        min_value = arr[i];
        min_index = i;
      }
    }
    this->minVal = min_value;
    this->minIdx = min_index;
}

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

float calculate_rewards(float current_position[], float previous_position[], float goal_position[], int action){
  float reward = 0.0;
  if(eucliden_distance(current_position, goal_position) <= 0.05 && action == 0) {
    reward += 50.0;
  }
  else if(eucliden_distance(current_position, goal_position) <= 0.1 ) {
    reward += 10.0;
  }
  else {
    float previous_distance = eucliden_distance(previous_position, goal_position);
    float current_distance  = eucliden_distance(current_position, goal_position);
    if(current_distance < previous_distance) {
      // reward += 5.0;
      reward += 0.5 * current_distance;
    }
    else {
      // reward -= 5.0;
      reward -= 5.0 * current_distance;
    }

    if((action == 3)||(action == 4)) {
      reward -= 5.0;
    }
  }

  cout << "current position (x,y) = (" << current_position[0] << "," << current_position[1] << ")" << " , Distance = " << eucliden_distance(current_position, goal_position);
  return reward;
}

float *calculate_action_values(double reward[], double action_count[]){
  float *action_values = new float[5];

  /// calculate action_values :
  for(int i=0; i<5; i++){
    if(action_count[i] == 0.0) {
      action_values[i] = 0.0;
    }
    else {
      action_values[i] = reward[i] / action_count[i];
    }
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "FuckingNode");
  TurtleBot3 robot;
  ros::Rate rate(1);

  double rewards[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double action_count[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  float action_values_arr[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  float epsilon = 0.05;
  int randomStatus = 0;
  int action;

  float current_position[2];
  float goal_position[] = {-1.0, 0.0};

  // float previous_distance = robot.get_laser(0);
  float previous_position[2];
  previous_position[0] = robot.get_position(1);
  previous_position[1] = robot.get_position(2);

  maxFind maxInfo;
  minFind minInfo;

  int episodCounter = 0;

  while(ros::ok()){
    float rand = randUniform(0.0, 1.0);
  
    if(rand < epsilon) {
      // select a random acttion :
      action = randInt(0, 4);
      randomStatus = 1;
    }
    else {
      randomStatus = 0;
      float *action_values_ptr = calculate_action_values(rewards, action_count);
      
      for(int i=0; i<5; i++){
        // action_values_arr[i] = *(action_values_ptr+i);
        action_values_arr[i] = *(action_values_ptr+i) + rewards[i];
        // action_values_arr[i] = rewards[i];
      }
      // select optimal action :
      maxInfo.maxFunc<float>(action_values_arr, 5);
      action = maxInfo.maxIdx;
    }

    if (action == 0) {
      // do nothing :
      robot.stop_moving();
    }
    else if (action == 1) {
      // move forward :
      robot.move_forward_meters(0.2);
    }
    else if (action == 2) {
      // move backward :
      robot.move_backward_meters(0.2);
    }
    else if (action == 3) {
      // turn right :
      robot.turn_degree(45.0);
      // robot.stop_moving();
    }
    else if (action == 4) {
      // turn left :
      robot.turn_degree(-45.0);
      // robot.stop_moving();
    }
    else {
      // ROS_INFO_STREAM("Unrecognized Action ");
      break;
      }

    // int maxDegrees = 360;
    // float *collision_ptr = robot.get_laser_full();
    // float collision_arr[maxDegrees];
    
    // for(int i=0; i<maxDegrees; i++) {
    //   collision_arr[i] = *(collision_ptr + i);
    // }

    
    // minInfo.minFunc<float>(collision_arr, maxDegrees);
    
    // double nearest_obstacle_distance  = minInfo.minVal;
    // int    nearest_obstacle_direction = minInfo.minIdx;

    current_position[0] = robot.get_position(1);
    current_position[1] = robot.get_position(2);


    // float reward = calculate_rewards(current_position, previous_position, goal_position, nearest_obstacle_distance, action);
    float reward = calculate_rewards(current_position, previous_position, goal_position, action);
    rewards[action] = rewards[action] + reward;
    action_count[action] = action_count[action] + 1.0;

    // rate.sleep();
    
    cout << endl << episodCounter++ << endl;
    ROS_INFO_STREAM("gift reward = " << reward << " , selected action = " << action << " , Random? " << randomStatus);
    ROS_INFO_STREAM("0" << " Action Value  = " << action_values_arr[0] << " , Action Count = " << action_count[0] << " , Reward = " << rewards[0]);
    ROS_INFO_STREAM("1" << " Action Value  = " << action_values_arr[1] << " , Action Count = " << action_count[1] << " , Reward = " << rewards[1]);
    ROS_INFO_STREAM("2" << " Action Value  = " << action_values_arr[2] << " , Action Count = " << action_count[2] << " , Reward = " << rewards[2]);
    ROS_INFO_STREAM("3" << " Action Value  = " << action_values_arr[3] << " , Action Count = " << action_count[3] << " , Reward = " << rewards[3]);
    ROS_INFO_STREAM("4" << " Action Value  = " << action_values_arr[4] << " , Action Count = " << action_count[4] << " , Reward = " << rewards[4]);

  }
  return 0;
} 
