filename = '../online_svm_data/oakland_part3_am_rf.node_features';
delimiterIn = ' ';
headerlinesIn = 3;

% 
A = dlmread(filename, delimiterIn, headerlinesIn);

%% Break table into classes
% 1004: Veg
% 1100: Wire
% 1103: Pole
% 1200: Ground
% 1400: Facade

label_col = 5;
% feat_col_start = 6;

veg_class = A(A(:,label_col)==1004, :);    % 8322 x 15
wire_class = A(A(:,label_col)==1100, :);   % 818 x 15
pole_class = A(A(:,label_col)==1103, :);   % 1429 x 15
ground_class = A(A(:,label_col)==1200, :); % 67161 x 15
facade_class = A(A(:,label_col)==1400, :); % 12092 x 15

size(veg_class,1) + size(wire_class,1) + size(pole_class,1) ...
    + size(ground_class,1) + size(facade_class,1)  % should be 89822

class1 = veg_class;
class2 = wire_class;

classes = [1004, 1100, 1103, 1200, 1400];
names = {'Veg', 'Wire', 'Pole', 'Ground', 'Facade'};

iter = 1;
for i=1:numel(classes)
    %for j=1:numel(classes) 
    for j=i+1:numel(classes) 
        
        class1 = A(A(:,label_col)==classes(i), :);
        class2 = A(A(:,label_col)==classes(j), :);
        
        %names{i}
        %names{j}
        
        [accuracy, misclass] = online_svm(class1, class2, iter, ...
            names{i}, names{j})
        iter = iter + 1;
    end    
end

figure
%showPointCloud(A(:,1:3), [0.9 0.9 0.9])
veg = A(A(:,5)==1004, :);
showPointCloud(veg(:,1:3), [0 1 0]);
hold on
wire = A(A(:,5)==1100, :);
showPointCloud(wire(:,1:3), [1 1 0]);
hold on
pole = A(A(:,5)==1103, :);
showPointCloud(pole(:,1:3), [1 0 1]);
hold on
ground = A(A(:,5)==1200, :);
showPointCloud(ground(:,1:3), [0 0 0]);
hold on
facade = A(A(:,5)==1400, :);
showPointCloud(facade(:,1:3), [1 0 0]);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Point cloud of the whole scene');
legend('Veg','Wire','Pole','Ground','Facade');