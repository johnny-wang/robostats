%% Naive plots
load('success_ratio.mat')

num_k = 8;

figure
plot(1:num_k, test_success_ratio)

hold on;
plot(1:num_k, train_success_ratio)
xlabel('Trajectories')
ylabel('Success Ratio')
legend('Test Data', 'Train Data')
title('Naive Prediction Strategy')

%% List Prediction plots
load('list_success_ratio.mat')

figure
hold on;
plot(1:num_k, list_test_success_ratio)

plot(1:num_k, list_train_success_ratio)
xlabel('Trajectories')
ylabel('Success Ratio')
legend('Test Data', 'Train Data')
title('List Prediction Strategy')