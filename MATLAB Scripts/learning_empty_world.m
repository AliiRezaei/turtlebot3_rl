clc
clear
close all

%% Create Spaces and Pairs

% x space :
x.min = -2.5;
x.max = +2.5;
x.n   = 11;
x.space_precise = (x.max - x.min) / (x.n - 1);
x.vec = linspace(x.min, x.max, x.n);

% y space :
y.min = -2.5;
y.max = +2.5;
y.n   = 11;
y.space_precise = (y.max - y.min) / (y.n - 1);
y.vec = linspace(y.min, y.max, y.n);

% theta space :
theta.min = -pi;
theta.max = +pi;
theta.n   = 9;
theta.space_precise = (theta.max - theta.min) / (theta.n - 1);
theta.vec = linspace(theta.min, theta.max, theta.n);

% create state space :
[first_col, second_col, third_col] = ndgrid(x.vec, y.vec, theta.vec);
all_states = [first_col(:), second_col(:), third_col(:)];
n_all_states = size(all_states, 1);
n_columns = size(all_states, 2);


%% Declare Actions and State-Action Pairs

% possible actions :
n_actions = 3;
actions = [0; 1; 2];

% create state action pairs :
[first_col, second_col, third_col, fourth_col] = ndgrid(x.vec, y.vec, theta.vec, actions);
state_action_pairs = [first_col(:), second_col(:), third_col(:), fourth_col(:)];
n_state_action_pairs = size(state_action_pairs, 1);
n_columns_action_pairs = size(state_action_pairs, 2);

%% Initialization

% goal point :
goal = [-0.5, 2.5];

% for store optimal policy :
policy = zeros(n_all_states, 1);

% for store Q Table :
QTable = zeros(n_state_action_pairs, 1);

% Q learning algorithm params :
n_episodes = 30000;
epsilon    = 0.9;
gamma      = 0.99;
alpha      = 0.1;

%% Q learning Algorithm

% start time :
tic;

% learning loop :
for e=1:n_episodes
    % select a random row :
    random_row = randi(n_all_states - 1);

    % state in episode :
    state = all_states(random_row, :);

    % reset rpe :
    rpe = 0;

    % main loop :
    while true
        state_idx = find(ismember(all_states, state, 'rows'));

        % epsilonilon greedy :
        if rand > epsilon
            action = policy(state_idx);
        else
            action = actions(randi(n_actions));
        end

        % do selected action :
        state_next = do_action(state, action, x, y, theta);

        % get reward and check new state status :
        [reward, state_next] = get_reward(all_states, state_next, state, goal);

        % find state_action_paired_idx that maximized q table :
        state_action_paired_idx = find(ismember(state_action_pairs, [state, action], 'rows'));
        state_action_paired_next_idx = find(ismember(state_action_pairs(:, 1:3), state_next, 'rows'));
        
        % update QTable :
        max_QTable = max(QTable(state_action_paired_next_idx)) - QTable(state_action_paired_idx);
        QTable(state_action_paired_idx) = QTable(state_action_paired_idx) + alpha * (reward + gamma * max_QTable);
        
        % improve policy :
        [~, argmax] = max(QTable(state_action_paired_next_idx));
        policy(state_action_paired_idx) = argmax - 1;
        % for i = 1:n_all_states
        %     [~, argmax] = max(QTable(1+n_actions*(i-1):n_actions*i));
        %     policy(i) = argmax;
        % end

        % update state :
        state = state_next;

        % reward per episode :
        rpe = rpe + reward;

        % check reaching condition :
        if all(state(1:2) == goal)
            break;
        end
    end
    disp(['Episode : ', num2str(e), ' Reward Per Episode : ', num2str(rpe)]);
    epsilon = epsilon * 0.97;
end

% time duration :
toc;

%% Local Functions

function state_next = do_action(state, action, x_space, y_space, theta_space)

    % exract states :
    x     = state(1);
    y     = state(2);
    theta = state(3);

    % space precises :
    x_space_precise     = x_space.space_precise;
    y_space_precise     = y_space.space_precise;
    theta_space_precise = theta_space.space_precise;

    epsilon = 0.01;
    if abs(theta + pi) < epsilon
        r1 = - 1.0;
        r2 =   0.0;
    elseif abs(theta + 3 * pi / 4.0) < epsilon
        r1 = - 1.0;
        r2 = - 1.0;
    elseif abs(theta + pi / 2.0) < epsilon
        r1 =   0.0;
        r2 = - 1.0;
    elseif abs(theta + pi / 4.0) < epsilon
        r1 =   1.0;
        r2 = - 1.0;
    elseif abs(theta) < epsilon
        r1 =   1.0;
        r2 =   0.0;
    elseif abs(theta - pi / 4.0) < epsilon
        r1 =   1.0;
        r2 =   1.0;
    elseif abs(theta - pi / 2.0) < epsilon
        r1 =   0.0;
        r2 =   1.0;
    elseif abs(theta - 3 * pi / 4.0) < epsilon
        r1 = - 1.0;
        r2 =   1.0;
    elseif abs(theta - pi) < epsilon
        r1 = - 1.0;
        r2 =   0.0;
    else
        disp("Are You Joking?!");
    end

    switch (action)
        case 0
            x_new = x + x_space_precise * r1;
            y_new = y + y_space_precise * r2;
            theta_new = theta;
    
        case 1
            x_new = x;
            y_new = y;
            theta_new = theta - theta_space_precise;
    
        case 2
            x_new = x;
            y_new = y;
            theta_new = theta + theta_space_precise;

        otherwise
            x_new = x;
            y_new = y;
            theta_new = theta;
    end

    % update state :
    state_next = zeros(1, 3);
    state_next(1) = x_new;
    state_next(2) = y_new;
    state_next(3) = theta_new;
end

function [reward, new_state] = get_reward(all_states, state_next, state, goal)
    state_status = find(ismember(all_states, state_next, 'rows')); %#ok
    new_state = state_next;
    if ~isempty(state_status)
        if all(state_next(1:2) == goal)
          reward = +10.0;
        else
          reward = -1.0;
        end
    else
        new_state = state;
        reward = -10.0;
    end
end