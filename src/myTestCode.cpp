// #include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "geometry_msgs/Twist.h"
// #include "sensor_msgs/LaserScan.h"
// #include "nav_msgs/Odometry.h"
// #include <string.h>
// #include <math.h>
// #include <cmath>
// #include <random>
// #include <iostream>
// using namespace std;


// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "FuckingNode");
//   TurtleBot3 robot;
//   ros::Rate rate(1);

//   double rewards[] = {0.0, 0.0, 0.0, 0.0, 0.0};
//   double action_count[] = {0.0, 0.0, 0.0, 0.0, 0.0};
//   float action_values_arr[] = {0.0, 0.0, 0.0, 0.0, 0.0};
//   float epsilon = 0.05;
//   int randomStatus = 0;
//   int action;

//   float current_position[2];
//   float goal_position[] = {-1.0, 0.0};

//   // float previous_distance = robot.get_laser(0);
//   float previous_position[2];
//   previous_position[0] = robot.get_position(1);
//   previous_position[1] = robot.get_position(2);

//   maxFind maxInfo;
//   minFind minInfo;

//   int episodCounter = 0;

//   while(ros::ok()){
//     float rand = randUniform(0.0, 1.0);
  
//     if(rand < epsilon) {
//       // select a random acttion :
//       action = randInt(0, 4);
//       randomStatus = 1;
//     }
//     else {
//       randomStatus = 0;
//       float *action_values_ptr = calculate_action_values(rewards, action_count);
      
//       for(int i=0; i<5; i++){
//         // action_values_arr[i] = *(action_values_ptr+i);
//         action_values_arr[i] = *(action_values_ptr+i) + rewards[i];
//         // action_values_arr[i] = rewards[i];
//       }
//       // select optimal action :
//       maxInfo.maxFunc<float>(action_values_arr, 5);
//       action = maxInfo.maxIdx;
//     }

//     if (action == 0) {
//       // do nothing :
//       robot.stop_moving();
//     }
//     else if (action == 1) {
//       // move forward :
//       robot.move_forward_meters(0.2);
//     }
//     else if (action == 2) {
//       // move backward :
//       robot.move_backward_meters(0.2);
//     }
//     else if (action == 3) {
//       // turn right :
//       robot.turn_degree(45.0);
//       // robot.stop_moving();
//     }
//     else if (action == 4) {
//       // turn left :
//       robot.turn_degree(-45.0);
//       // robot.stop_moving();
//     }
//     else {
//       // ROS_INFO_STREAM("Unrecognized Action ");
//       break;
//       }

//     // int maxDegrees = 360;
//     // float *collision_ptr = robot.get_laser_full();
//     // float collision_arr[maxDegrees];
    
//     // for(int i=0; i<maxDegrees; i++) {
//     //   collision_arr[i] = *(collision_ptr + i);
//     // }

    
//     // minInfo.minFunc<float>(collision_arr, maxDegrees);
    
//     // double nearest_obstacle_distance  = minInfo.minVal;
//     // int    nearest_obstacle_direction = minInfo.minIdx;

//     current_position[0] = robot.get_position(1);
//     current_position[1] = robot.get_position(2);


//     // float reward = calculate_rewards(current_position, previous_position, goal_position, nearest_obstacle_distance, action);
//     float reward = calculate_rewards(current_position, previous_position, goal_position, action);
//     rewards[action] = rewards[action] + reward;
//     action_count[action] = action_count[action] + 1.0;

//     // rate.sleep();
    
//     cout << endl << episodCounter++ << endl;
//     ROS_INFO_STREAM("gift reward = " << reward << " , selected action = " << action << " , Random? " << randomStatus);
//     ROS_INFO_STREAM("0" << " Action Value  = " << action_values_arr[0] << " , Action Count = " << action_count[0] << " , Reward = " << rewards[0]);
//     ROS_INFO_STREAM("1" << " Action Value  = " << action_values_arr[1] << " , Action Count = " << action_count[1] << " , Reward = " << rewards[1]);
//     ROS_INFO_STREAM("2" << " Action Value  = " << action_values_arr[2] << " , Action Count = " << action_count[2] << " , Reward = " << rewards[2]);
//     ROS_INFO_STREAM("3" << " Action Value  = " << action_values_arr[3] << " , Action Count = " << action_count[3] << " , Reward = " << rewards[3]);
//     ROS_INFO_STREAM("4" << " Action Value  = " << action_values_arr[4] << " , Action Count = " << action_count[4] << " , Reward = " << rewards[4]);

//   }
//   return 0;
// } 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "turtlebot3_rl/TurtleMove.h"
#include "turtlebot3_rl/Toolbox.h"
#include <iostream>

int main(int argc, char **argv) {

    ros::init(argc, argv, "test_go_target_node");

    TurtleBot3 robot;

    robot.move_forward_meters(1.0);
    robot.turn_in_radians(M_PI / 2.0);

    robot.move_forward_meters(1.0);
    robot.turn_in_radians(M_PI / 2.0);
    
    robot.move_forward_meters(1.0);
    robot.turn_in_radians(M_PI / 2.0);
    
    robot.move_forward_meters(1.0);

    robot.stop_moving();

    return 0;
}
