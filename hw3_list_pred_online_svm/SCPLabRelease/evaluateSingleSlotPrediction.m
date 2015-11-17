% Input:
% score_test: column vector of predicted score. Bigger score implies higher 
% confidence in predicting the trajectory will succeed.  
% result_test: ground truth result of whether its trajectory would succeed
% for its corresponding environment. 

function [success_envs] = evaluateSingleSlotPrediction(score_test, result_test)
num_traj_per_env = 30;
num_environments_test = 212;
scores_test_reshaped = reshape(score_test', num_traj_per_env, num_environments_test)';
[vals, idx] = max(scores_test_reshaped, [], 2);
success_envs = find(result_test(sub2ind([num_traj_per_env, num_environments_test], idx', 1:num_environments_test)) == 1);
fprintf('Ratio of success:%f\n',length(success_envs) / num_environments_test);
end

