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
    int *actions = rl.create_actions(n_actions);

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
    theta.max =  2.0 * 3.14159265;
    theta.n   =  4;

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

    // pairing states and actions :
    float **state_action_pairs = rl.create_state_action_pairs(all_states, actions);
    std::size_t n_state_action_pairs   = n_all_states * n_actions;
    std::size_t n_columns_action_pairs = n_columns + 1;

    // // test created pairing atates and actions :
    // for(std::size_t i = 0; i < n_state_action_pairs; i++) {
    //   for(std::size_t j = 0; j < n_columns_action_pairs; j++) {
    //     std::cout << *(*(state_action_pairs + i) + j) << "\t";
    //   }
    // std::cout << std::endl;
    // }

    // learning params :
    int   n_episodes = 1;
    float gamma      = 0.99;
    float alpha      = 0.10;
    struct Interaction int_with_env;

    // goal selection :
    float  goal_state[] = {1.0, 0.0};
    float *goal = goal_state;

    // initialize polices :
    int *policy = rl.get_random_policy(n_all_states);
    // for(int i=0; i<n_all_states; i++) {
    //     std::cout << *(policy + i) << std::endl;
    // }

    

    // init current and next state pointers :
    float *state = new float[n_columns];
    float *state_next = new float[n_columns];

    // init state index with None :
    int state_idx = -1; 

    // init action with None :
    int action = -1;

    // init a rand var with None :
    float rand = -1.0;

    // init reward per episode (befor starts learning, reward is zero):
    float reward = 0.0;

    // learning loop :
    for(int e=0; e<n_episodes; e++) {
        int random_row = rl.randGenerator.randInteger(0, n_all_states - 1);
        
        for(std::size_t i=0; i<n_columns; i++) {
            *(state + i) = *(*(all_states + random_row) + i);
        }
        
        while(e++<n_episodes) {
            // find random selected state in all_states :
            state_idx = ismember<float>(state, all_states, n_all_states, n_columns);
            
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
            int_with_env = rl.do_action(all_states, state, state_next, action, goal);
            state_next   = int_with_env.state_next;
            reward       = int_with_env.reward;

            std::cout << *(state_next + 0) << "\t" << *(state + 0) << std::endl;
            std::cout << *(state_next + 1) << "\t" << *(state + 1) << std::endl;
            std::cout << *(state_next + 2) << "\t" << *(state + 2) << std::endl;
            // for(std::size_t i=0; i<n_columns; i++) {
            //      std::cout << *(state_next + i) << "\t" << *(state + i) << "\t";
            // }
            std::cout << reward << std::endl;
            std::cout << action << std::endl;


            // int state_action_paired_idx = ismember<float>(state_action_check_ptr, state_action_pairs_ptr, n_state_action_pairs, n_columns_action_pairs);
            // int state_action_paired_next_idx = ismember<float>(state_next, state_action_pairs_ptr, n_state_action_pairs, n_columns);
            // std::cout << state_action_paired_idx << "\t" << state_action_paired_next_idx << std::endl;
        }
        
        

    }



    return 0;
}