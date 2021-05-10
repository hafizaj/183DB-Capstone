% 3 DOF Forward Kinematics
% Adapted from https://github.com/akshaykvnit/3dof-planar-robotic-arm-simulation/
% Taking input before showing initial positions.
% Parameters t1, t2, t3 are the joint angles.
% Outputs coordinates
%% Robot Dimensions
clf;clear;clc;
%Length of segments and gripper
inch_to_m = 0.0254;
L1 = 20*inch_to_m;  %length of the first segment 
L2 = 20*inch_to_m;  %length of the second segment
L3 = 4*inch_to_m; %length of the third segment 
b = 10*inch_to_m;     %length of the gripper
L4 = L3+b/2;
% Initial positions of the joints of the 3 arms
x = [0 L1 L1+L2 L4+L1+L2];
y = [0 0 0 0];

%%
t1_ = 20;
t2_ = 40;
t3_ = 60;

t1 = t1_ * (pi / 180);
t2 = t2_ * (pi / 180);
t3 = t3_ * (pi / 180);

%%
% angles between previous and next x axes are denoted by theta.
theta1 = t1;
theta2 = t2;
theta3 = t3;
theta = t1 * t2 * t3;
if theta < 0
    theta = -theta;
end

% Product of all 3 angles will be a multiple of LCM of 3 angles
theta = theta1 * theta2 * theta3;

% Accounting for negative values of angles.
if theta < 0
    theta = -theta;
end
%% Plotting
% Running a for loop to obtain animation
X_arr = [];
Y_arr = [];
V_arr = []; %angular velocities for all joints
A_arr = []; %angular acceleration for all joints
Angle_arr =[]; %angular position of joints
time = 3;


for i = 0 : 0.001 : time
    [tt1,ang_vel1,ang_acc1] = cubic_polynomial(theta1,time,i);
    [tt2,ang_vel2,ang_acc2] = cubic_polynomial(theta2,time,i);
    [tt3,ang_vel3,ang_acc3] = cubic_polynomial(theta3,time,i);
    
    ang_vel = [ang_vel1, ang_vel2, ang_vel3];
    ang_acc = [ang_acc1, ang_acc2, ang_acc3];
    angle_arr = [tt1, tt2, tt3];

    % transformation matrices
    Y_temp = [];
    X_temp = [];
    % rotation matrices
    R1 = Rotate(tt1);
    R2 = Rotate(tt2);
    R3 = Rotate(tt3);

    % translation matrices
    % parameters are link lengths
    T2 = Translate(L1);
    T3 = Translate(L2);
    T4 = Translate(L4);

    % finding second point

    % transformation matrix
    Y_iter1 = R1 * T2;

    % Finding new coordinates
    Y1 = Y_iter1 * [0; 0; 0; 1];
    x(2) = Y1(1);
    y(2) = Y1(2);

    % finding third point

    % transformation matrix
    Y_iter2 = R1 * T2 * R2 * T3;

    % Finding new coordinates
    Y2 = Y_iter2 * [0; 0; 0; 1];
    x(3) = Y2(1);
    y(3) = Y2(2);

    % finding fourth point

    % transformation matrix
    Y_iter3= R1 * T2 * R2 * T3 * R3 * T4;

    % Finding new coordinates
    Y3 = Y_iter3 * [0; 0; 0; 1];
    x(4) = Y3(1);
    y(4) = Y3(2);

    X_arr = [X_arr ;x];
    Y_arr = [Y_arr ;y];
    V_arr = [V_arr ;ang_vel];
    A_arr = [A_arr ;ang_acc];
    Angle_arr =[Angle_arr; angle_arr];
    
end

%%

T = 0:0.001:time;
for i = 1:1:3 
    figure(i)
    subplot(3,1,1)
    plot(T, rad2deg(Angle_arr(:,i)));
    hold on;
    title(sprintf('Angular position vs Time for joint: %.0f ', i));
    xlabel('time (s)'); ylabel('Ang Pos. (deg)');
    grid on;
    subplot(3,1,2)
    plot(T,V_arr(:,i));
    hold on;
    title(sprintf('Angular velocity vs Time for joint: %.0f ', i));
    xlabel('time (s)'); ylabel('Ang. Vel. (rad/s)');
    grid on;
    subplot(3,1,3)
    plot(T,A_arr(:,i));
    hold on;
    title(sprintf('Angular acceleration vs Time for joint: %.0f ', i));
    xlabel('time (s)'); ylabel('Ang. Acc. (rad/s)');
    grid on;
    
end

%%




% figure(1)
% plot(T,X_arr(:,4));
% figure(2);
% plot(T,V_arr(:,3));
% figure(3);
% plot(T,A_arr(:,3));
% figure(4); 
% plot(T, rad2deg(Angle_arr(:,3)));



%% 
function [ R ] = Rotate( theta )
%Rotate Returns rotation matrix
%   Input angle theta is used to create 3x3 rotation matrix
R = [cos(theta) -sin(theta) 0 0;
    sin(theta) cos(theta) 0 0;
    0 0 1 0
    0 0 0 1];
end

%%
function [ T ] = Translate( d )
%Translate Returns translation matrix
%   Returns 3x3 translation matrix using displacement d.

T = [1 0 0 d;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
end