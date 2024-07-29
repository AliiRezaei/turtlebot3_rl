#ifndef _ReinforcementLearning_
#define _ReinforcementLearning_

#include "Toolbox.h"
#include <iostream>

class QLearning {
  private:
    int    n_actions;
    
    float  epsilon;

    minFind minInfo;
    maxFind maxInfo;

    randomGen randGenerator;

    float x_space_precise;
    float y_space_precise;
    float theta_space_precise;

    
    
  public:


    // int   *Actions = new int[n_actions];
    int   *Actions = new int[5];
    
    QLearning(int n_actions, float epsilon);

    void   create_actions();
    std::vector<std::vector<float>>   create_states(stateInfo x, stateInfo y, stateInfo theta);
    int    get_random_action(int *actions_set);
    int    get_best_action(float *action_values, int *actions_set);
    int    epsilon_greedy(double reward[], int *actions_set, double actions_count[]);
    float  get_rewards(float current_position[], float previous_position[], float goal_position[], int action);
    float *get_action_values(double reward[], double actions_count[]);
    
};

QLearning::QLearning(int n_actions, float epsilon) {
    this->n_actions = n_actions;
    this->epsilon   = epsilon;

    randomGen randGenerator;
    this->randGenerator = randGenerator;

    minFind minInfo;
    this->minInfo = minInfo;

    maxFind maxInfo;
    this->maxInfo = maxInfo;
    
    create_actions();
    // create_states();
}

void QLearning::create_actions() {
    for(int i=0; i<n_actions; i++) {
        *(Actions + i) = i;
    }
}

std::vector<std::vector<float>> QLearning::create_states(stateInfo x, stateInfo y, stateInfo theta) {
  // extract info :
  float x_min = x.min;
  float x_max = x.max;
  int   x_n   = x.n;
  this->x_space_precise = (x_max - x_min) / (x_n - 1.0);

  float y_min = y.min;
  float y_max = y.max;
  int   y_n   = y.n;
  this->y_space_precise = (y_max - y_min) / (y_n - 1.0);

  float theta_min = theta.min;
  float theta_max = theta.max;
  int   theta_n   = theta.n;
  this->theta_space_precise = (theta_max - theta_min) / (theta_n - 1.0);

  // x space :
  float *x_ = linspace<float>(x_min, x_max, x_n);
  // this->x_space = x_;
  
  // y space :
  float *y_ = linspace<float>(y_min, y_max, y_n);
  // this->y_space = y_;

  // theta space :
  float *theta_ = linspace<float>(theta_min, theta_max, theta_n);
  // this->theta_space = theta_;

  // create augmented states matrix :
  std::size_t all_states_row = x_n * y_n * theta_n;
  std::size_t all_states_col = 3;

  std::vector<std::vector<float>> augmented_states(all_states_row, std::vector<float>(all_states_col));
  for(std::size_t i = 0; i < x_n; i++) {
    for(std::size_t j = 0; j < y_n; j++) {
      for(std::size_t k = 0; k < theta_n; k++) {
        std::size_t index = i * (y_n * theta_n) + j * theta_n + k;
        augmented_states[index][0] = x_[i];
        augmented_states[index][1] = y_[j];
        augmented_states[index][2] = theta_[k];
      }
    }
  }
  delete[] x_;
  delete[] y_;
  delete[] theta_;
  return augmented_states;
}

int QLearning::get_random_action(int *actions_set) {
  
  minInfo.minFunc(actions_set, n_actions);
  int min_actions =  minInfo.minVal;
  
  maxInfo.maxFunc(actions_set, n_actions);
  int max_actions =  maxInfo.maxVal;
  
  int random_action = randGenerator.randInteger(min_actions, max_actions);
  
  return random_action;
}

int QLearning::get_best_action(float *action_values, int *actions_set) {
  
  maxInfo.maxFunc(action_values, n_actions);
  int max_action_value =  maxInfo.maxVal;
  int max_action_index =  maxInfo.maxIdx;
  
  int best_action = *(actions_set + max_action_index);

  return best_action;
  // return max_action_index;
}

int QLearning::epsilon_greedy(double reward[], int *actions_set, double actions_count[]) {
  
  // uniform random number :
  float rand = randGenerator.randUniform(0.0, 1.0);
  
  // None action :
  int action = -1;

  // exploitation and exploration : 
  if(rand > epsilon) {
    float *action_values = get_action_values(reward, actions_count);
    action = get_best_action(action_values, actions_set);
  }
  else {
    action = get_random_action(this->Actions);
  }
   return action; 
}

float QLearning::get_rewards(float current_position[], float previous_position[], float goal_position[], int action) {
  
  float reward = 0.0;
  float previous_distance = eucliden_distance(previous_position, goal_position);
  float current_distance  = eucliden_distance(current_position, goal_position);

  if(action == 0 && current_distance > 0.05) {
    reward -= 10.0;
  }
  if(current_distance <= 0.05 && action == 0) {
    reward += 50.0;
  }
  if(current_distance < previous_distance) {
    reward += 1.0 * current_distance;
    // reward += 1.0;
  }
  else {
    reward -= 2.0 * current_distance;
  }
  if(action == 3 || action == 4) {
    reward -= 1.0;
  }
  return reward;
}

float *QLearning::get_action_values(double reward[], double actions_count[]){
  float *action_values = new float[5];

  /// calculate action_values :
  for(int i=0; i<5; i++){
    if(actions_count[i] == 0.0) {
      action_values[i] = 0.0;
    }
    else {
      action_values[i] = reward[i] / actions_count[i];
      // action_values[i] = reward[i];
    }
  }
  return action_values;
}

#endif