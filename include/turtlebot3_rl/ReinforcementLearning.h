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

    void   set_actions();
    
  public:


    int   *Actions = new int[n_actions];
    
    QLearning(int n_actions, float epsilon);
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
    
    set_actions();
}

void QLearning::set_actions() {
    for(int i=0; i<n_actions; i++) {
        *(Actions + i) = i;
    }
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
  
  minInfo.minFunc(action_values, n_actions);
  int min_actions_value =  minInfo.minVal;
  int min_actions_index =  minInfo.minIdx;
  
  int best_action = *(actions_set + min_actions_index);

  return best_action;
}

int QLearning::epsilon_greedy(double reward[], int *actions_set, double actions_count[]) {
  
  // uniform random number :
  float rand = randGenerator.randUniform(0.0, 1.0);
  
  // None action :
  int action = -1;

  // exploitation and exploration : 
  if(rand < epsilon) {
    float *action_values = get_action_values(reward, actions_count);
    action = get_best_action(action_values, actions_set);
  }
  else {
    action = get_random_action(Actions);
  }
   return action; 
}

float QLearning::get_rewards(float current_position[], float previous_position[], float goal_position[], int action){
  float reward = 0.0;
  if(eucliden_distance(current_position, goal_position) <= 0.05 && action == 0) {
    reward += 50.0;
  }
  else if(eucliden_distance(current_position, goal_position) <= 0.1 ) {
    reward += 10.0;
  }
  else {
    float previous_distance = eucliden_distance(previous_position, goal_position);
    float current_distance  = eucliden_distance(current_position, goal_position);
    if(current_distance < previous_distance) {
      // reward += 5.0;
      reward += 0.5 * current_distance;
    }
    else {
      // reward -= 5.0;
      reward -= 5.0 * current_distance;
    }

    if((action == 3)||(action == 4)) {
      reward -= 5.0;
    }
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
    }
  }
  cout << endl;
  return action_values;
}

#endif