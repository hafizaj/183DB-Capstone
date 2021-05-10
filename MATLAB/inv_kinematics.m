function [theta_1,theta_2,theta_3] = inv_kinematics(x_goal,y_goal,phi_goal)
l1 = 20;  %length of the first segment 
l2 = 20;  %length of the second segment
l3 = 4; %length of the third segment in inches
b = 10;  %length of the gripper in inches
l4 = l3+b/2;    
x1 = x_goal - l4*cos(phi_goal);
y1 = y_goal - l4*sin(phi_goal);
A = -2*l1*x1;
B = -2*l1*y1; 

alpha = atan2(B/(sqrt(A^2+B^2)), A/(sqrt(A^2+B^2)));
sign = [1,-1];
theta1 = alpha + sign*acos(-(x1^2+y1^2+l1^2-l2^2)/(2*l1*sqrt(x1^2+y1^2)));
theta2 = sign*acos((x1^2+y1^2-l1^2-l2^2)/(2*l1*l2));
theta3 = phi_goal-theta1-theta2;

theta_1 = rad2deg(theta1(1));
theta_2 = rad2deg(theta2(1));
theta_3 = rad2deg(theta3(1));
end