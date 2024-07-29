#include "turtlebot3_rl/Toolbox.h"
#include "turtlebot3_rl/ReinforcementLearning.h"
#include <iostream>
#include <vector>

int main() {
    int   n_actions = 5;
    float epsilon   = 0.05;

    // rl object :
    QLearning rl(n_actions, epsilon);

    // state space info :
    struct stateInfo x;
    x.min = -1.0;
    x.max = +1.0;
    x.n   =  10;
    
    struct stateInfo y;
    y.min = -1.0;
    y.max = +1.0;
    y.n   =  10;
    
    struct stateInfo theta;
    theta.min = -1.0;
    theta.max = +1.0;
    theta.n   =  10;

    // create states space :
    std::vector<std::vector<float>> all_states = rl.create_states(x, y, theta);
    for(std::size_t i=0; i<x.n*y.n*theta.n; i++) {
        std::cout << all_states[i][0] << "\t" << all_states[i][1] << "\t" << all_states[i][2] << "\t" << std::endl;
    }


    return 0;
}