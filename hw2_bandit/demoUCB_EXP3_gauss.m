%% This script applies a random policy on a gaussian game
clc;
close; 
clear all;

%% Get the gaussian game
num_actions = 10;
num_rounds = 10000;
game = gameGaussian(num_actions, num_rounds);

%% Get a set of policies to try out
policies = {policyEXP3(), policyUCB()};
policy_names = {'policyEXP3', 'policyUCB'};

%% Run the policies on the game
f1 = figure;
hold on;
f2 = figure;
hold on;

for k = 1:length(policies)
    policy = policies{k};
    game.resetGame();
    [reward, action, regret] = game.play(policy);
    fprintf('Policy: %s Reward: %.2f\n', class(policy), sum(reward));
    
    % Plot regret of the policies on its own graph
    figure(f1);
    plot(regret);
    
    if length(policies) > 1
        % scatter plot the actions since random will jump around
        figure(f2);
        subplot(2,1,k);
        scatter(1:length(action), action);
        legend(policy_names(k));
    else
        figure(f2);
        scatter(1:length(action), action);
        legend(policy_names(k));
    end
    xlabel('Rounds');
    ylabel('Action');
    title('Action vs Rounds of Games');
end
figure(f1);
legend(policy_names);
xlabel('Rounds');
ylabel('Regret');
title('Regret vs Rounds of Games');
