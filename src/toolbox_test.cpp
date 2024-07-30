#include "turtlebot3_rl/Toolbox.h"
#include "turtlebot3_rl/ReinforcementLearning.h"
#include <iostream>
#include <vector>

int main() {

    int   n_actions = 5;
    float epsilon   = 0.9;

    // rl object :
    QLearning rl(n_actions, epsilon);

    // create actions :
    int *actions = rl.create_actions(n_actions);

    // state space info :
    struct stateInfo x;
    x.min = -5.0;
    x.max = +5.0;
    x.n   =  11;
    
    struct stateInfo y;
    y.min = -5.0;
    y.max = +5.0;
    y.n   =  11;

    struct stateInfo theta;
    theta.min =  0.0;
    theta.max =  2.0 * _PI_NUMBER_;
    theta.n   =  5;

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
    int   n_episodes = 500;
    float gamma      = 0.990;
    float alpha      = 0.100;
    struct Interaction int_with_env;

    // goal selection :
    float  goal_state[] = {0.0, 0.0};
    float *goal = goal_state;

    // initialize polices :
    int *policy = rl.get_random_policy(n_all_states);
    // for(int i=0; i<n_all_states; i++) {
    //     std::cout << *(policy + i) << std::endl;
    // }

    // initialize q table :
    float *Qtable = rl.zeros_init_Qtable(n_state_action_pairs);
    // float *Qtable = rl.random_init_Qtable(n_state_action_pairs, 0.0, 1.0);
    // for(int i=0; i<n_state_action_pairs; i++) {
    //     std::cout << *(Qtable + i) << std::endl;
    // }

    // initialize q table window for calculate q table in state s for all actions :
    float *Qtable_window = new float[n_actions];

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

    // init paired state and action in each episode :
    float *state_action_paired = new float[n_columns_action_pairs];
    

    // init state_action_paired_idx for calculat the best idx that maximized q table :
    int  state_action_paired_idx;

    // init state_action_paired_next_idx for calculat the best idx of next in q table :
    int *state_action_paired_next_idx = new int[n_actions];

    // max finder object :
    maxFind max_of_Qtable;

    // init reward per episode var :
    float rpe = 0.0;

    int i = 0;
    // learning loop :
    for(int e=0; e<n_episodes; e++) {
        
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
            reward = rl.do_action(all_states, state, state_next, action, goal);

            // // test do_action correct works or not :
            // std::cout << *(state_next + 0) << "\t" << *(state + 0) << std::endl;
            // std::cout << *(state_next + 1) << "\t" << *(state + 1) << std::endl;
            // std::cout << *(state_next + 2) << "\t" << *(state + 2) << std::endl;
            // std::cout << reward << std::endl;
            // std::cout << action << std::endl;


            // create state_action_paired at this state for all actions:
            for(i=0; i<n_columns; i++) {
                *(state_action_paired + i) = *(state + i);
            }
            *(state_action_paired + n_columns) = action;

            // // test state_action_paired :
            // for(int i=0; i<n_actions; i++) {
            //     for(int j=0; j<n_columns_action_pairs; j++) {
            //         std::cout << *(*(state_action_paired + i) + j) << "\t";
            //     }
            //     std::cout << std::endl;
            // }

            // find state_action_paired_idx that maximized q table :
            state_action_paired_idx = ismember<float>(state_action_paired, state_action_pairs, n_state_action_pairs, n_columns_action_pairs);
            *state_action_paired_next_idx = ismember<float>(state_next, state_action_pairs, n_state_action_pairs, n_columns);
            // std::cout << *state_action_paired_next_idx << std::endl;
            
            for(i=0; i<n_actions; i++) {
                *(Qtable_window + i) = Qtable[*(state_action_paired_next_idx) + i];
            }

            max_of_Qtable.maxFunc(Qtable_window, n_actions);
            Qtable[state_action_paired_idx] = Qtable[state_action_paired_idx] + alpha * (reward + gamma * max_of_Qtable.maxVal - Qtable[state_action_paired_idx]);
            // std::cout << Qtable[*state_action_paired_idx] << std::endl;
            // std::cout << Qtable[state_action_paired_idx] << std::endl;

            // policy improvement :
            rl.improve_policy(Qtable, n_state_action_pairs, policy);
            
            // for(int i=0; i<n_all_states; i++) {
            //     std::cout << *(policy + i) << std::endl;
            // }
            // std::cout << *(policy + 0) << std::endl;

            // std::cout << *(state + 0) << "\t" << *(state + 1) << "\t" << *(state + 2) << "\t \t \t"  << *(state_next + 0) << "\t" << *(state_next + 1) << "\t" << *(state_next + 2) << std::endl;

            // update state :
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
        std::cout << "Episode : " << e << "\t" << "Reward per Episode : " << rpe << std::endl;
        epsilon = epsilon * 0.97;
    }
    // for(int i=0; i<n_state_action_pairs; i++) {
    //     std::cout << *(Qtable + i) << std::endl;
    // }
    return 0;
}