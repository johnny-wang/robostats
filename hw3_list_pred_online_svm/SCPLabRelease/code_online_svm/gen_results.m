num_perms = 10;

for i=1:num_perms
% for i=1:2

    fname = sprintf('online_svm_log_%d.mat', i)
    load(fname);
    
    % Show the error rate
    figure;
    plot(1:num_rows, err_rate);
    xlabel('Number of points (rounds) processed');
    ylabel('Percent classification error (1 = 100%)');
    str = sprintf('Classification error by round of %s vs %s', class1_name, class2_name);
    title(str);
    
    % Show the 3D point cloud of the two classes
    figure;
%     size(predictions);
%     size(perm_classes);
    size(perm_classes(predictions==1))
    size(perm_classes(predictions==-1))
    showPointCloud(perm_classes(predictions==1, 1:3), 'green');
    hold on
    showPointCloud(perm_classes(predictions==-1, 1:3), 'red');
    hold on
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    str = sprintf('Point cloud of %s vs %s', class1_name, class2_name);
    title(str);
    legend(class1_name, class2_name);
    
end