function [theta_t,angular_vel,angular_acc] = cubic_polynomial(theta_f, t_f, t)
%constraints:
theta_0 = 0;
%theta_f - joint angle calculated from inverse kinematics
% dot_theta_0 = 0; %initial velocity is zero 
% dot_theta_f = 0 %final velocity is zero 
a0 = theta_0;
a1 = 0; 
a2 = (3/t_f^2)*(theta_f - theta_0);
a3 = (-2/t_f^3)*(theta_f - theta_0);

theta_t = a0 + a1*t + a2*t^2 + a3*t^3;
angular_vel = a1 + 2*a2*t + 3*a3*t^2;
angular_acc = 2*a2 + 6*a3*t
end 

