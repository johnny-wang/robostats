
% load the data
load('../data.mat');

% concatenate all the training/test features/results
all_examples = [feat_train; feat_test];
all_results = [result_train; result_test];

all_b = all_examples \ all_results;
all_ans = all_examples * all_b;

all_feat_M = reshape(all_ans, numTrajPerEnv, ...
                numEnvironmentsTrain + numEnvironmentsTest);
all_res_M = reshape(all_results, numTrajPerEnv, ...
                numEnvironmentsTrain + numEnvironmentsTest);
            
num_k = 8;            

% add column of ones for bias
test_ones = ones(numTrajPerEnv * numEnvironmentsTest, 1);
train_ones = ones(numTrajPerEnv * numEnvironmentsTrain, 1);

train_with_ones = [feat_train train_ones];
test_with_ones = [feat_test test_ones];

%% Using the Test data

% Find the weight vector
b = test_with_ones \ result_test;

% multiply weight with test result to get estimated confidence
%test_ans = feat_test * b;
test_ans = test_with_ones * b;

% number of Trajectories is numTrajPerEnv = 30
% number of columns is numEnvironmentsTest = 212

test_M = reshape(test_ans, numTrajPerEnv, numEnvironmentsTest);

% also reshape results into columns of 30
result_M = reshape(result_test, numTrajPerEnv, numEnvironmentsTest);

% number of successful environments with k trajectories (1-8)
% each row is the number of successful env at k trajectory.
% row1 = (k=1), row2 = (k=2), row3 = (k=3), etc.
% 8 x 212 for Test
k_success = zeros(num_k, numEnvironmentsTest);

% sort max to min
%[sorted_vals, sorted_idx] = sort(test_M(:,1), 'descend')
%test_M(sorted_idx(1),1)
%result_M(sorted_idx(1),1)

for i=1:numEnvironmentsTest
    for k=1:num_k
        [sorted_vals, sorted_idx] = sort(test_M(:,i), 'descend');

        sorted_idx(1:k);
        %sorted_vals(k)
        
        test_M(sorted_idx(1:k),i);
        % value(s) of 1 and 0 indicating if trajectory was successful
        is_success = result_M(sorted_idx(1:k),i);
        any(is_success);
        k_success(k, i) = any(is_success);
    end
end

% sum up each row to get the number of successes of k for total env
success_env = sum(k_success, 2)
test_success_ratio = success_env / numEnvironmentsTest

%% Using the Train data

% multiply weight with test result to get estimated confidence
%test_ans = feat_train * b;
test_ans = train_with_ones * b;

% number of Trajectories is numTrajPerEnv = 30
% number of columns is numEnvironmentsTrain = 310
test_M = reshape(test_ans, numTrajPerEnv, numEnvironmentsTrain);

% also reshape results into columns of 30
result_M = reshape(result_train, numTrajPerEnv, numEnvironmentsTrain);

% 8 x 310 for Train
k_success = zeros(num_k, numEnvironmentsTrain);

for i=1:numEnvironmentsTrain
    for k=1:num_k
        [sorted_vals, sorted_idx] = sort(test_M(:,i), 'descend');
       
        sorted_idx(1:k);
        %sorted_vals(k)
        
        test_M(sorted_idx(1:k),i);
        % value(s) of 1 and 0 indicating if trajectory was successful
        is_success = result_M(sorted_idx(1:k),i);
        %any(is_success)
        k_success(k, i) = any(is_success);
    end
end

% sum up each row to get the number of successes of k for total env
success_env = sum(k_success, 2)
train_success_ratio = success_env / numEnvironmentsTrain

save('success_ratio.mat', 'test_success_ratio', 'train_success_ratio')
