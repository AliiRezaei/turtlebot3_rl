#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "turtlebot3_rl/TurtleMove.h"
#include "turtlebot3_rl/Toolbox.h"
#include "turtlebot3_rl/ReinforcementLearning.h"
#include <iostream>

int main(int argc, char **argv) {

    // // init node :
    // ros::init(argc, argv, "FuckingNode");
    
    // // TurtleBot3 object :
    // TurtleBot3 robot;

    // // QLearning object :
    // QLearning rl(5, 0.05);
    
    // // initial rewards :
    // double rewards[] = {0.0, 0.0, 0.0, 0.0, 0.0};

    // // initial actions_count :
    // double actions_count[] = {0.0, 0.0, 0.0, 0.0, 0.0};

    // // robot previous position :
    // float previous_position[2];
    // previous_position[0] = robot.get_position(1); // x pos
    // previous_position[1] = robot.get_position(2); // y pos

    // // robot current position :
    // float current_position[2];
    // current_position[0] = robot.get_position(1); // x pos
    // current_position[1] = robot.get_position(2); // y pos

    // // goal position :
    // float goal_position[] = {-1.0, 0.0};

    // // episod counter :
    // int episod_counter = 0;

    // // loop rate :
    // ros::Rate rate(10);
    
    // // main loop :
    // while(ros::ok()){
    //     // int action = rl.epsilon_greedy(rewards, rl.Actions, actions_count);
    //     int action = rl.epsilon_greedy(rewards, rl.actions, actions_count);

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
    //     }

    //     // update current position :
    //     current_position[0] = robot.get_position(1);
    //     current_position[1] = robot.get_position(2);

    //     // get reward at this point :
    //     float reward = rl.get_rewards(current_position, previous_position, goal_position, action);
        
    //     // get action values :
    //     float *action_values = rl.get_action_values(rewards, actions_count);
        
    //     // update rewards :
    //     rewards[action] = rewards[action] + reward;

    //     // update actions count :
    //     actions_count[action] = actions_count[action] + 1.0;

    //     // log info :
    //     cout << endl << episod_counter++ << endl;
    //     ROS_INFO_STREAM("gift reward = " << reward << " , selected action = " << action);
    //     ROS_INFO_STREAM("0" << " Action Value  = " << *(action_values + 0) << " , Action Count = " << actions_count[0] << " , Reward = " << rewards[0]);
    //     ROS_INFO_STREAM("1" << " Action Value  = " << *(action_values + 1) << " , Action Count = " << actions_count[1] << " , Reward = " << rewards[1]);
    //     ROS_INFO_STREAM("2" << " Action Value  = " << *(action_values + 2) << " , Action Count = " << actions_count[2] << " , Reward = " << rewards[2]);
    //     ROS_INFO_STREAM("3" << " Action Value  = " << *(action_values + 3) << " , Action Count = " << actions_count[3] << " , Reward = " << rewards[3]);
    //     ROS_INFO_STREAM("4" << " Action Value  = " << *(action_values + 4) << " , Action Count = " << actions_count[4] << " , Reward = " << rewards[4]);

    //     rate.sleep();
    // }
    return 0;
}


