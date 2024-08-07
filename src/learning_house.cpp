#include "turtlebot3_rl/Toolbox.h"
#include "turtlebot3_rl/ReinforcementLearning.h"
#include "turtlebot3_rl/TurtleMove.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>

int main(int argc, char **argv) {

    int   n_actions = 3;
    float epsilon   = 0.9;

    // rl object :
    QLearning rl(n_actions, epsilon);

    // create actions :
    int *actions = rl.create_actions(n_actions);

    // state space info :
    struct stateInfo x;
    x.min = -6.5;
    x.max = -3.0;
    x.n   =  8;
    
    struct stateInfo y;
    y.min = +0.5;
    y.max = +3.5;
    y.n   =  7;

    struct stateInfo theta;
    theta.min = - _PI_NUMBER_;
    theta.max =   _PI_NUMBER_;
    theta.n   =  9;

    // create states space :
    float **all_states = rl.create_states(x, y, theta);
    std::size_t n_all_states = x.n * y.n * theta.n;
    std::size_t n_columns = 3;

    // get collision points :
    float **collisions = rl.get_collisions("house");
    int n_collisions = 5 * theta.n;

    // pairing states and actions :
    float **state_action_pairs = rl.create_state_action_pairs(all_states, actions);
    std::size_t n_state_action_pairs   = n_all_states * n_actions;
    std::size_t n_columns_action_pairs = n_columns + 1;

    // learning params :
    int   n_episodes = 40000;
    float gamma      = 0.990;
    float alpha      = 0.100;

    // goal selection :
    float  goal_state[] = {-6.5, 0.5};
    float *goal = goal_state;

    // initialize policy :
    int *policy = rl.get_random_policy(n_all_states);

    // initialize q table :
    float *Qtable = rl.zeros_init_Qtable(n_state_action_pairs);

    // initialize q table window for calculate q table in state s for all actions :
    float *Qtable_window = new float[n_actions];

    // init current and next state pointers :
    float *state = new float[n_columns];
    float *state_next = new float[n_columns];

    // init state index with None :
    int state_idx = -1;

    // collision_status for check state is exist in collision set or not :
    int collision_status = -1; 

    // init action with None :
    int action = -1;

    // init a rand var with None :
    float rand = -1.0;

    // init reward per episode (befor starts learning, reward is zero):
    float reward = 0.0;

    // init paired state and action in each episode :
    float *state_action_paired = new float[n_columns_action_pairs];
    
    // init state_action_paired_idx for calculat the best idx that maximized q table :
    int  state_action_paired_idx;

    // init state_action_paired_next_idx for calculat the best idx of next in q table :
    int *state_action_paired_next_idx = new int[n_actions];

    // max finder object :
    maxFind max_of_Qtable;

    // reward per episode :
    float rpe = 0.0;

    // learning process start time :
    auto start_time = std::chrono::high_resolution_clock::now();

    // maybe used in loop :
    int i = 0;

    // learning loop :
    for(int e=1; e<=n_episodes; e++) {
        
        // select a random state :
        int random_row = rl.randGenerator.randInteger(0, n_all_states - 1);
        for(i=0; i<n_columns; i++) {
            *(state + i) = *(*(all_states + random_row) + i);
        }
        
        // reset rpe :
        rpe = 0.0;
        while(1) {
            // find random selected state in all_states :
            state_idx = ismember<float>(state, all_states, n_all_states, n_columns);
            // std::cout << state_idx << std::endl;

            // check selected state is exist in collision set or not :
            collision_status = ismember<float>(state, collisions, n_collisions, n_columns);
            if(collision_status != -1) { // selected state is exist in collision set
                break;
            }
            else { // selected state is not exist in collision set
                // select an action :
                rand = rl.randGenerator.randUniform(0.0, 1.0);
                if(rand > epsilon) {
                    // select an action from optimal policy :
                    action = *(policy + state_idx);
                }
                else {
                    // select a random action :
                    action = rl.get_random_action(rl.actions);
                }

                // doing selected action and get reward and new state :
                rl.do_action(state, state_next, action);

                // calculate reward :
                reward = rl.get_reward(all_states, state_next, state, goal, action);

                // create state_action_paired at this state for all actions:
                for(i=0; i<n_columns; i++) {
                    *(state_action_paired + i) = *(state + i);
                }
                *(state_action_paired + n_columns) = action;

                // find state_action_paired_idx that maximized q table :
                state_action_paired_idx = ismember<float>(state_action_paired, state_action_pairs, n_state_action_pairs, n_columns_action_pairs);
                *state_action_paired_next_idx = ismember<float>(state_next, state_action_pairs, n_state_action_pairs, n_columns);
                // std::cout << *state_action_paired_next_idx << std::endl;

                for(i=0; i<n_actions; i++) {
                    *(Qtable_window + i) = Qtable[*(state_action_paired_next_idx) + i];
                }

                max_of_Qtable.maxFunc(Qtable_window, n_actions);
                Qtable[state_action_paired_idx] = Qtable[state_action_paired_idx] + alpha * (reward + gamma * max_of_Qtable.maxVal - Qtable[state_action_paired_idx]);

                // policy improvement :
                rl.improve_policy(Qtable, n_state_action_pairs, policy);

                // update states :
                *(state + 0) = *(state_next + 0);
                *(state + 1) = *(state_next + 1);
                *(state + 2) = *(state_next + 2);

                // some of rewards :
                rpe += reward;

                // check reach goal state :
                if(*(state + 0) == *(goal + 0) && *(state + 1) == *(goal + 1)) {
                  break;
                }
            }
        }
        std::cout << "Episode : " << e << "\t" << "Reward per Episode : " << rpe << std::endl;
        epsilon = epsilon * 0.97;
    }

    // learning process end time :
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;

    // display learning process time taken :
    std::cout << "Learning process has been completed in " << duration.count() << " seconds!" << std::endl;

    // // create a text file for log learning process :
    // std::ofstream learning_data;

    // // file path :
    // learning_data.open("/home/ali/catkin_ws/src/turtlebot3_rl/LogData/data_house.txt");
    
    // // log algorithm params :
    // learning_data << "Gamma     = " << gamma << " , " << "alpha    = " << alpha << " , " << "episodes            = " << n_episodes << std::endl;
    // learning_data << "n actions = " << n_actions << "    , " << "n states = " << n_all_states << " , " << "n state action pair = " << n_state_action_pairs << std::endl;
    // learning_data << std::endl;
    // learning_data << "row" << "\t \t " << "x" << "\t \t"  << " y" << "\t \t"  << "theta" << "\t \t"  << "actions" << "\t \t"  << "best action" << "\t \t" << "Q Table" << std::endl;
    
    // // log q table and optimal policy :
    // int index=0; // for log optimal policy
    // for(int i=0; i<n_state_action_pairs; i++) {
    //     learning_data << i << "\t \t" << *(*(state_action_pairs + i) + 0) << "\t \t" << *(*(state_action_pairs + i) + 1) << "\t \t" << std::setprecision(4) << *(*(state_action_pairs + i) + 2) << "\t \t" << *(*(state_action_pairs + i) + 3) << "\t \t" << *(policy + index) << "\t \t \t" << std::setprecision(4) << *(Qtable + i) << std::endl;
    //     if(((i + 1) % n_actions) == 0) {index++;}
    // }

    // // save and close file:
    // learning_data.close();

    // init node :
    ros::init(argc, argv, "main_house_node");
    
    // TurtleBot3 object :
    TurtleBot3 robot;

    // start pose :
    float p[] = {-3.0, 1.0, 0.0};
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