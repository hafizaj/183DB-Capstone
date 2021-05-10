clf; 
clear; 
clc;
%end_effector coordinates
% x_goal = 39;   
y_goal = 10; 
phi_goal = 0.9;
x_goal = 20;
X =[];
Y =[];
Phi = [];
i = 1;
theta1 =[];
theta2 =[];
theta3 =[];
ret_x =[];
ret_y =[];
ret_phi =[];
while x_goal<21
 
    %use inverse kinematics to calculate joint angles
    [theta1(i),theta2(i),theta3(i)] = inv_kinematics(x_goal,y_goal,phi_goal);

    %use forward kinematics to get to end effector
    [ret_x(i),ret_y(i), ret_phi(i)] = fk_final(theta1(i),theta2(i),theta3(i));
    X(i) =x_goal;
    Y(i) =y_goal;
    Phi(i) = rad2deg(phi_goal);
  
if (x_goal == 0)
    break
else 
    x_goal= x_goal-1;
end
i = i+1;
end

table = [X', Y', Phi', theta1', theta2',theta3', ret_x', ret_y', ret_phi'];
table = array2table(table,'variablenames',{'x_goal','y_goal','phi_goal','theta1','theta2','theta3','ret_x','ret_y','ret_phi'});
disp(table);

%% 


% while i<180    %setting the first joint not to exceed 180 deg
%     j = -9;
%     while j<9 
%         fk_final(i,j,15);   
%     j = j+1;
%     end
% i=i+1;
% end

%%
\
    end
    end
end 
