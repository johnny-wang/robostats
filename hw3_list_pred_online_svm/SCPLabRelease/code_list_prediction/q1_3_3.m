% load the data
load('../data.mat');

% add column of ones for bias
test_ones = ones(numTrajPerEnv * numEnvironmentsTest, 1);
train_ones = ones(numTrajPerEnv * numEnvironmentsTrain, 1);

train_with_ones = [feat_train train_ones];
test_with_ones = [feat_test test_ones];

all_examples = [train_with_ones; test_with_ones]; % 15660 x 18
all_results = [result_train; result_test];  % 15660 x 1

num_k = 8;            

% 1. Training a new learner for a new slot with updated features and labels
% 2. Predicting the marginal utilities for all items given features and
%    pick the best
% 3. Labels update
% 4. Optional features update

% Each learner is 18x1
learners = zeros(18, num_k);

total_features = numEnvironmentsTrain + numEnvironmentsTest;

% We will keep updating the result matrix every loop
updated_results = all_results;

% Train 8 learners
for j=1:num_k
    
    % Generate new learner from the given feature and updated results
    % all_examples = 15660 x 18
    % updated_results = 15660 x 1

    b = all_examples \ updated_results;  % 18x1    
    learners(:,j) = b;
    
    all_ans = all_examples * b;

    % number of Trajectories is numTrajPerEnv = 30
    % number of columns is numEnvironmentsTrain = 310
    % number of columns is numEnvironmentsTest = 212
    % 30 x 512
    all_feat_M = reshape(all_ans, numTrajPerEnv, total_features);
    all_res_M = reshape(updated_results, numTrajPerEnv, total_features);
    
    % column of zeros to update marginal benefit of result
    col_zeros = zeros(numTrajPerEnv,1);

    for i=1:total_features
       [val,idx] = max(all_feat_M(:,i));
       %all_res_M(idx,i)
       if(all_res_M(idx,i))
           %updated_results
           all_res_M(:,i) = col_zeros;
       end
    end

    all_vals_stacked = total_features * numTrajPerEnv;  %15560
    updated_results = reshape(all_res_M, all_vals_stacked, 1); % 15560 x 1

end

% 30 x 522
updated_results = reshape(updated_results, numTrajPerEnv, ...
    numEnvironmentsTrain + numEnvironmentsTest);

%% Test

% reshape results into columns of 30
result_M = reshape(result_test, numTrajPerEnv, numEnvironmentsTest);

% Store success (1) of environment
k_success = zeros(num_k,numEnvironmentsTest);
list_test_success_ratio = zeros(1,num_k);

for j=1:num_k
    
    test_k = test_with_ones * learners(:,j);   % 6360 x 1
    test_k = reshape(test_k, numTrajPerEnv, numEnvironmentsTest); % 30 x 212

    for i=1:numEnvironmentsTest  
       [val, idx] = max(test_k(:,i));
       k_success(j,i) = result_M(idx,i);
    end

    % 1 if any element in column is 1
    success_cols = any(k_success, 1);
    list_test_success_ratio(j) = sum(success_cols) / numEnvironmentsTest;
end

%% Train

% reshape results into columns of 30
result_M = reshape(result_train, numTrajPerEnv, numEnvironmentsTrain);

% Store success (1) of environment
k_success = zeros(num_k,numEnvironmentsTrain);
list_train_success_ratio = zeros(1,num_k);

for j=1:num_k
    
    test_k = train_with_ones * learners(:,j);   % 9300 x 1   
    test_k = reshape(test_k, numTrajPerEnv, numEnvironmentsTrain); % 30 x 310
    
    for i=1:numEnvironmentsTrain
       [val, idx] = max(test_k(:,i));
       k_success(j,i) = result_M(idx,i);
    end

    % 1 if any element in column is 1
    success_cols = any(k_success, 1);
    list_train_success_ratio(j) = sum(success_cols) / numEnvironmentsTrain;
end

save('list_success_ratio.mat', 'list_test_success_ratio', 'list_train_success_ratio')
