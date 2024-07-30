#include "turtlebot3_rl/Toolbox.h"
#include "turtlebot3_rl/ReinforcementLearning.h"
#include <iostream>
#include <vector>

int main() {

    int   n_actions = 5;
    float epsilon   = 0.05;

    // rl object :
    QLearning rl(n_actions, epsilon);

    // create actions :
    std::vector<int> actions = rl.create_actions(n_actions);

    // state space info :
    struct stateInfo x;
    x.min = -1.0;
    x.max = +1.0;
    x.n   =  2;
    
    struct stateInfo y;
    y.min = -1.0;
    y.max = +1.0;
    y.n   =  2;
    
    struct stateInfo theta;
    theta.min =  0.0;
    theta.max = 3.14159265;
    theta.n   =  2;

    // create states space :
    float **all_states = rl.create_states(x, y, theta);
    std::size_t n_all_states = x.n * y.n * theta.n;
    std::size_t n_columns = 3;

    // // test created states space :
    // for(std::size_t i = 0; i < n_all_states; i++) {
    //   for(std::size_t j = 0; j < n_columns; j++) {
    //     std::cout << *(*(all_states + i) + j) << "\t";
    //   }
    // std::cout << std::endl;
    // }

    // // test equavalency of **a == *a[] ok, the result is true
    // float s[] = {-1.0, -1.0, 0.0};
    // float *state = s;
    // int state_idx = ismember<float>(state, all_states, n_all_states, n_columns);
    // std::cout << state_idx << std::endl;





    // // temp test :
    // float all_states_temp[n_all_states][n_columns];
    // for(std::size_t i=0; i<n_all_states; i++) {
    //     for(std::size_t j=0; j<n_columns; j++) {
    //         all_states_temp[i][j] = all_states[i][j];
    //     }
    // }

    // float *all_states_ptr[n_all_states];
    // for(std::size_t i=0; i<n_all_states; i++) {
    //     all_states_ptr[i] = all_states_temp[i];
    //     // std::cout << all_states_ptr[i] << std::endl;
    // }
    // // for(std::size_t i=0; i<x.n*y.n*theta.n; i++) {
    // //     std::cout << all_states[i][0] << "\t" << all_states[i][1] << "\t" << all_states[i][2] << "\t" << std::endl;
    // // }




    // // pairing atates and actions :
    // std::vector<std::vector<float>> state_action_pairs = rl.create_state_action_pairs(all_states, actions);
    // std::size_t n_state_action_pairs   = n_all_states * theta.n;
    // std::size_t n_columns_action_pairs = n_columns + 1;
    
    // float state_action_pairs_temp[n_state_action_pairs][n_columns_action_pairs];
    // for(std::size_t i=0; i<n_state_action_pairs; i++) {
    //     for(std::size_t j=0; j<n_columns_action_pairs; j++) {
    //         state_action_pairs_temp[i][j] = state_action_pairs[i][j];
    //         // std::cout << state_action_pairs_temp[i][j] << std::endl;
    //     }
    // }

    // float *state_action_pairs_ptr[n_state_action_pairs];
    // for(std::size_t i=0; i<n_state_action_pairs; i++) {
    //     state_action_pairs_ptr[i] = state_action_pairs_temp[i];
    //     // std::cout << state_action_pairs_ptr[i] << std::endl;
    // }
    // // for(std::size_t i=0; i<x.n*y.n*theta.n*n_actions; i++) {
    // //     std::cout << state_action_pairs[i][0] << "\t" << state_action_pairs[i][1] << "\t" << state_action_pairs[i][2] << "\t" << state_action_pairs[i][3] << "\t" << std::endl;
    // // }


    // // learning params :
    // int   n_episodes = 1;
    // float gamma      = 0.99;
    // float alpha      = 0.10;
    // struct Interaction int_with_env;

    // // goal selection :
    // float  goal_state[] = {1.0, 0.0};
    // float *goal = goal_state;

    // // initialize polices :
    // int *policy = rl.get_random_policy(n_all_states);
    // // for(int i=0; i<n_all_states; i++) {
    // //     std::cout << *(policy + i) << std::endl;
    // // }

    // // learning loop :
    // for(int e=0; e<n_episodes; e++) {
    //     int random_row = rl.randGenerator.randInteger(0, n_all_states - 1);
    //     float *state = new float[n_columns];
    //     float *state_next = new float[n_columns];
    //     for(std::size_t i=0; i<n_columns; i++) {
    //         *(state + i) = all_states[random_row][i];
    //     }
        
    //     while(e++<n_episodes) {
    //         int state_idx = ismember<float>(state, all_states_ptr, n_all_states, n_columns);
    //         float rand = rl.randGenerator.randUniform(0.0, 1.0);
    //         int action = -1; // None acttion
    //         if(rand > epsilon) {
    //             action = *(policy + state_idx);
    //         }
    //         else {
    //             action = rl.get_random_action(rl.actions);
    //         }
    //         // float *state_next = rl.do_action(state, action);
    //         int_with_env = rl.do_action(all_states_ptr, state, action, goal);
    //         state_next = int_with_env.state_next;

    //         float reward = int_with_env.reward;

    //         // std::cout << *(state_next + 0) << "\t" << *(state + 0) << std::endl;
    //         // std::cout << *(state_next + 1) << "\t" << *(state + 1) << std::endl;
    //         // std::cout << *(state_next + 2) << "\t" << *(state + 2) << std::endl;
    //         // for(std::size_t i=0; i<n_columns; i++) {
    //         //      std::cout << *(state_next + i) << "\t" << *(state + i) << "\t";
    //         // }
    //         // std::cout << reward << std::endl;
    //         // std::cout << action << std::endl;


    //         // float  state_action_check[n_columns_action_pairs];
    //         // float *state_action_check_ptr = state_action_check;
    //         // *(state_action_check_ptr + 0) = *(state + 0);
    //         // *(state_action_check_ptr + 1) = *(state + 1);
    //         // *(state_action_check_ptr + 2) = *(state + 2);
    //         // *(state_action_check_ptr + 3) = (float) action;

    //         // int state_action_paired_idx = ismember<float>(state_action_check_ptr, state_action_pairs_ptr, n_state_action_pairs, n_columns_action_pairs);
    //         // int state_action_paired_next_idx = ismember<float>(state_next, state_action_pairs_ptr, n_state_action_pairs, n_columns);
    //         // std::cout << state_action_paired_idx << "\t" << state_action_paired_next_idx << std::endl;
    //     }
        
        

    // }



    return 0;
}