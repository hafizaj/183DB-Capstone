function [ret_x,ret_y, ret_phi] = fk_final(t1,t2,t3)
    inch_to_m = 1;
    L1 = 20*inch_to_m;  %length of the first segment 
    L2 = 20*inch_to_m;  %length of the second segment
    L3 = 4*inch_to_m; %length of the third segment 
    b = 10*inch_to_m;     %length of the gripper
    L4 = L3+b/2;
    % Initial positions of the joints of the 3 arms
    x = [0 L1 L1+L2 L4+L1+L2];
    y = [0 0 0 0];
    

    % Fix axes
    axlimits = L1 + L2 + L4 +2;
    axis([-axlimits axlimits -axlimits axlimits]);

    % I am assuming the reference frames' x axes along the length of
    % the links for each link.

    t1 = t1 * (pi / 180);
    t2 = t2 * (pi / 180);
    t3 = t3 * (pi / 180);

    % angles between previous and next x axes are denoted by theta.
    theta1 = t1;
    theta2 = t2;
    theta3 = t3;

    % Product of all 3 angles will be a multiple of LCM of 3 angles
    theta = theta1 * theta2 * theta3;

    % Accounting for negative values of angles.
    if theta < 0
        theta = -theta;
    end

    tt1 = theta1;
    tt2 = theta2;
    tt3 = theta3;
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
    Y = R1 * T2;
    % Finding new coordinates
    Y1 = Y * [0; 0; 0; 1];
    x(2) = Y1(1);
    y(2) = Y1(2);

    % finding third point
    % transformation matrix
    Y = R1 * T2 * R2 * T3;

    % Finding new coordinates
    Y1 = Y * [0; 0; 0; 1];
    x(3) = Y1(1);
    y(3) = Y1(2);

    % finding fourth point
    % transformation matrix
    Y = R1 * T2 * R2 * T3 * R3 * T4;

    % Finding new coordinates
    Y1 = Y * [0; 0; 0; 1];
    x(4) = Y1(1);
    y(4) = Y1(2);
    l = line(x,y); 
    hold on;
    l1 = plot(x(1), y(1), 'r.');
    l2 = plot(x(2), y(2), 'r.');
    l3 = plot(x(3), y(3), 'r.');
    l4 = plot(x(4), y(4), 'ro');
    l5 = plot(x(1), y(1), 'gs');
    l6 = plot(x(2), y(2), 'gs');
    l7 = plot(x(3), y(3), 'gs');
    l8 = plot(x(4), y(4), 'gs'); 
    axis([-axlimits axlimits -axlimits axlimits]);
    xlabel('x (inches)');
    ylabel('y (inches)');
  
    ret_x = x(4);
    ret_y = y(4);
    ret_phi = rad2deg(theta1+theta2+theta3);
    grid();
end 
