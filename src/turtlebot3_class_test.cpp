#include "turtlebot3_rl/TurtleMove.h"
#include "turtlebot3_rl/Toolbox.h"
#include <iostream>

int main(int argc, char **argv) {

    // ros init node  :
    ros::init(argc, argv, "turtlebot3_class_test_node");

    TurtleBot3 robot;

    // test move_forward_meters member function :  --> result : correct works.
    robot.move_forward_meters(1.0);
    robot.stop_moving();

    // test turn_in_radians member function :  --> result : correct works.
    robot.turn_in_radians(-M_PI / 2.0);
    robot.stop_moving();

    robot.move_forward_meters(1.0);
    robot.stop_moving();

    robot.move_backward_meters(1.0);
    robot.stop_moving();

    robot.turn_in_radians(M_PI / 2.0);
    robot.stop_moving();

    robot.move_backward_meters(1.0);
    robot.stop_moving();

    return 0;
}
