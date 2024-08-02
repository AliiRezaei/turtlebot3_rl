#include "turtlebot3_rl/Toolbox.h"
#include "turtlebot3_rl/ReinforcementLearning.h"
#include "turtlebot3_rl/TurtleMove.h"
#include <iostream>

int main(int argc, char **argv) {

    // init node :
    ros::init(argc, argv, "main_empty_world_node");
    
    // TurtleBot3 object :
    TurtleBot3 robot;

    // start pose :
    float p[] = {0.0, 0.0, 0.0};
    float *pose = p;

    float x_test;
    float y_test;
    float theta_test;

    float x_new_test;
    float y_new_test;
    float theta_new_test;

    float x_space_precise_test = (x.max - x.min) / (x.n - 1.0);
    float y_space_precise_test = (y.max - y.min) / (y.n - 1.0);
    float theta_space_precise_test = (theta.max - theta.min) / (theta.n - 1.0);

    while(ros::ok()) {

        // search pose in all states :
        state_idx = ismember<float>(pose, all_states, n_all_states, n_columns);
        action = *(policy + state_idx);

        x_test     = *(pose + 0);
        y_test     = *(pose + 1);
        theta_test = *(pose + 2);   
        
        float r1;
        float r2;
        float K;

        float eps = 0.01;

        if(fabs(theta_test + _PI_NUMBER_) < eps) {
          r1 = - 1.0;
          r2 =   0.0;
        }
        else if(fabs(theta_test + 3 * _PI_NUMBER_ / 4.0) < eps) {
          r1 = - 1.0;
          r2 = - 1.0;
        }
        else if(fabs(theta_test + _PI_NUMBER_ / 2.0) < eps) {
          r1 =   0.0;
          r2 = - 1.0;
        }
        else if(fabs(theta_test + _PI_NUMBER_ / 4.0) < eps) {
          r1 =   1.0;
          r2 = - 1.0;
        }
        else if(fabs(theta_test) < eps) {
          r1 =   1.0;
          r2 =   0.0;
        }
        else if(fabs(theta_test - _PI_NUMBER_ / 4.0) < eps) {
          r1 =   1.0;
          r2 =   1.0;
        }
        else if(fabs(theta_test - _PI_NUMBER_ / 2.0) < eps) {
          r1 =   0.0;
          r2 =   1.0;
        }
        else if(fabs(theta_test - 3 * _PI_NUMBER_ / 4.0) < eps) {
          r1 = - 1.0;
          r2 =   1.0;
        }
        else if(fabs(theta_test - _PI_NUMBER_) < eps) {
          r1 = - 1.0;
          r2 =   0.0;
        }
        else {
          std::cout << "Are You Joking?!" << std::endl;
        }   
        switch (action)
        {
        case 0:
          // move forward in theta direcction :
          x_new_test = x_test + x_space_precise_test * r1;
          y_new_test = y_test + y_space_precise_test * r2;
          theta_new_test = theta_test;
          robot.move_forward_meters(sqrt(r1*r1 + r2*r2) * x_space_precise_test);
          break;

        case 1:
          // turn cw :
          x_new_test = x_test;
          y_new_test = y_test;
          theta_new_test = theta_test - theta_space_precise_test;
          robot.turn_in_radians(-theta_space_precise_test);
          break;

        case 2:
          // turn ccw :
          x_new_test = x_test;
          y_new_test = y_test;
          theta_new_test = theta_test + theta_space_precise_test;
          robot.turn_in_radians(+theta_space_precise_test);
          break;

        default:
          x_new_test = x_test;
          y_new_test = y_test;
          theta_new_test = theta_test;
          break;
        }
        *(pose + 0) = x_new_test;
        *(pose + 1) = y_new_test;
        *(pose + 2) = theta_new_test;
        // check reach goal state :
        if(*(pose + 0) == *(goal + 0) && *(pose + 1) == *(goal + 1)) {
          break;
        }
    }
    return 0;
}