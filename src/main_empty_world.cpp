#include "turtlebot3_rl/Toolbox.h"
#include "turtlebot3_rl/ReinforcementLearning.h"
#include "turtlebot3_rl/TurtleMove.h"
#include <iostream>

int main(int argc, char **argv) {

    // // init node :
    // ros::init(argc, argv, "main_empty_world_node");
    
    // // TurtleBot3 object :
    // TurtleBot3 robot;

    // // start pose :
    // float p[] = {0.0, 0.0, 0.0};
    // float *pose = p;

    // float x_test;
    // float y_test;
    // float theta_test;

    // float x_new_test;
    // float y_new_test;
    // float theta_new_test;

    // float x_space_precise_test = (x.max - x.min) / (x.n - 1.0);
    // float y_space_precise_test = (y.max - y.min) / (y.n - 1.0);
    // float theta_space_precise_test = (theta.max - theta.min) / (theta.n - 1.0);

    // while(ros::ok()) {

    //     // search pose in all states :
    //     state_idx = ismember<float>(pose, all_states, n_all_states, n_columns);
    //     action = *(policy + state_idx);

    //     x_test     = *(pose + 0);
    //     y_test     = *(pose + 1);
    //     theta_test = *(pose + 2);   
        
    //     float r1;
    //     float r2;
    //     float K;

    //     float eps = 0.01;

    //     if(fabs(theta_test + _PI_NUMBER_) < eps) {
    //       r1 = - 1.0;
    //       r2 =   0.0;
    //     }
    //     else if(fabs(theta_test + 3 * _PI_NUMBER_ / 4.0) < eps) {
    //       r1 = - 1.0;
    //       r2 = - 1.0;
    //     }
    //     else if(fabs(theta_test + _PI_NUMBER_ / 2.0) < eps) {
    //       r1 =   0.0;
    //       r2 = - 1.0;
    //     }
    //     else if(fabs(theta_test + _PI_NUMBER_ / 4.0) < eps) {
    //       r1 =   1.0;
    //       r2 = - 1.0;
    //     }
    //     else if(fabs(theta_test) < eps) {
    //       r1 =   1.0;
    //       r2 =   0.0;
    //     }
    //     else if(fabs(theta_test - _PI_NUMBER_ / 4.0) < eps) {
    //       r1 =   1.0;
    //       r2 =   1.0;
    //     }
    //     else if(fabs(theta_test - _PI_NUMBER_ / 2.0) < eps) {
    //       r1 =   0.0;
    //       r2 =   1.0;
    //     }
    //     else if(fabs(theta_test - 3 * _PI_NUMBER_ / 4.0) < eps) {
    //       r1 = - 1.0;
    //       r2 =   1.0;
    //     }
    //     else if(fabs(theta_test - _PI_NUMBER_) < eps) {
    //       r1 = - 1.0;
    //       r2 =   0.0;
    //     }
    //     else {
    //       std::cout << "Are You Joking?!" << std::endl;
    //     }   
    //     switch (action)
    //     {
    //     case 0:
    //       // move forward in theta direcction :
    //       x_new_test = x_test + x_space_precise_test * r1;
    //       y_new_test = y_test + y_space_precise_test * r2;
    //       theta_new_test = theta_test;
    //       robot.move_forward_meters(sqrt(r1*r1 + r2*r2) * x_space_precise_test);
    //       break;

    //     case 1:
    //       // turn cw :
    //       x_new_test = x_test;
    //       y_new_test = y_test;
    //       theta_new_test = theta_test - theta_space_precise_test;
    //       robot.turn_in_radians(-theta_space_precise_test);
    //       break;

    //     case 2:
    //       // turn ccw :
    //       x_new_test = x_test;
    //       y_new_test = y_test;
    //       theta_new_test = theta_test + theta_space_precise_test;
    //       robot.turn_in_radians(+theta_space_precise_test);
    //       break;

    //     default:
    //       x_new_test = x_test;
    //       y_new_test = y_test;
    //       theta_new_test = theta_test;
    //       break;
    //     }
    //     *(pose + 0) = x_new_test;
    //     *(pose + 1) = y_new_test;
    //     *(pose + 2) = theta_new_test;
    //     // check reach goal state :
    //     if(*(pose + 0) == *(goal + 0) && *(pose + 1) == *(goal + 1)) {
    //       break;
    //     }
    // }
    return 0;
}

// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <vector>

// int main() {
//     // Variables to store header information
//     double gamma = 0.0, alpha = 0.0;
//     int episodes = 0, n_actions = 0, n_states = 0, n_state_action_pairs = 0;

//     // Vectors to store the columns
//     std::vector<int> row, actions, best_action;
//     std::vector<double> x, y, theta, q_table;

//     // Open the file
//     std::ifstream infile("/home/ali/catkin_ws/src/turtlebot3_rl/LogData/data_empty_world.txt");
//     std::string line;

//     // Read the header information
//     if (std::getline(infile, line)) {
//         std::stringstream ss(line);
//         std::string temp;

//         // Extract variables from the first line
//         ss >> temp >> gamma >> temp >> alpha >> temp >> temp >> episodes;
//         std::getline(infile, line);  // Skip the next line (column headers)
//         std::getline(infile, line);  // Read the third line with n actions, n states, etc.
//         std::stringstream ss2(line);
//         ss2 >> temp >> temp >> temp >> n_actions >> temp >> temp >> temp >> n_states >> temp >> temp >> temp >> n_state_action_pairs;
//     }

//     // Read the data rows
//     while (std::getline(infile, line)) {
//         std::stringstream ss(line);
//         int r, act, b_act;
//         double x_val, y_val, theta_val, q_val;

//         ss >> r >> x_val >> y_val >> theta_val >> act >> b_act >> q_val;

//         // Store data in vectors
//         row.push_back(r);
//         x.push_back(x_val);
//         y.push_back(y_val);
//         theta.push_back(theta_val);
//         actions.push_back(act);
//         best_action.push_back(b_act);
//         q_table.push_back(q_val);
//     }

//     infile.close();

//     // Now you can use the extracted data
//     // For example, printing some of the data:
//     std::cout << "Gamma: " << gamma << ", Alpha: " << alpha << ", Episodes: " << episodes << std::endl;
//     std::cout << "First row data: x=" << x[0] << ", y=" << y[0] << ", theta=" << theta[0] << std::endl;

//     return 0;
// }
