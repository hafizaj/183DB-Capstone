%%ECE183DB Storage Sub-System Simulation
wb_robot_init();
TIME_STEP = wb_robot_get_basic_time_step()
motor_horizontal = wb_robot_get_device('horizontal');
motor_vertical = wb_robot_get_device('vertical');
arm_vertical = wb_robot_get_device('arm_vertical');


a0 = wb_robot_get_device('a0');
P1 = wb_robot_get_device('P1');
P2 = wb_robot_get_device('P2');

wb_touch_sensor_enable(a0, TIME_STEP)
wb_position_sensor_enable(P1,TIME_STEP)
wb_position_sensor_enable(P2,TIME_STEP)


in_s = [1 1];

%%this is a general framework of the simulation. It will need to be
%%contained in a repetitive loop or in some manner if we have a defined set
%%of input

a = [0 0];
s = in_s;

wb_robot_step(TIME_STEP);
INIT_VERTICAL = wb_position_sensor_get_value(P1);
INIT_HORIZONTAL = wb_position_sensor_get_value(P2);
TOP_FLOOR = INIT_VERTICAL - 0.22;
BOTTOM_FLOOR = INIT_VERTICAL - 0.46;
LEFT_CLMN = INIT_HORIZONTAL + 0.58;
RIGHT_CLMN = INIT_HORIZONTAL + 0.3;

%k = 1;
while wb_robot_step(TIME_STEP) ~= -1
  a(1) = wb_touch_sensor_get_value(a0);
  if a(2)==1 %we received input from user that the storage is cleared
    s(1)=0;
  elseif (a(1)==0 || s(2)==0)
    %do nothing
  elseif (a(1)==1)
    if s(1) == 0 || s(1) == 1
      vert = TOP_FLOOR;
    else
      vert = BOTTOM_FLOOR;
    end
    if s(1) == 0 || s(1)==2
      hori = LEFT_CLMN;
    else
      hori = RIGHT_CLMN;
    end
    %WE NEED TO ADD BUFFER HERE TO GIVE TIME FOR THE
    %GRIPPER TO RELEASE TRAY 
    t = delay(500);
    
    v1 = 0.2;
    v2 = 0.1;
    v3 = 0.1; 

    %move column down
    while wb_position_sensor_get_value(P1)> vert %t <= TIME1
      wb_robot_step(TIME_STEP);
      wb_motor_set_position(motor_vertical, inf);
      wb_motor_set_velocity(motor_vertical,-v1);
      wb_motor_set_position(arm_vertical, inf);
      wb_motor_set_velocity(arm_vertical,-v1);
    end
    wb_motor_set_velocity(motor_vertical,0);
    %delay 0.5s
    delay(500);
    %move column left
    while wb_position_sensor_get_value(P2) < hori 
      wb_robot_step(TIME_STEP);
      wb_motor_set_position(motor_horizontal, inf);
      wb_motor_set_velocity(motor_horizontal,v2);
    end
    wb_motor_set_velocity(motor_horizontal,0);
    %move column down
    t = 1;
    while t <= (0.04/v3)*1000
      wb_robot_step(TIME_STEP);
      wb_motor_set_velocity(motor_vertical,-v3);
      t = t+1;
    end
    wb_motor_set_velocity(motor_vertical,0);
    %move column right

    while wb_position_sensor_get_value(P2) > INIT_HORIZONTAL
      wb_robot_step(TIME_STEP);
      wb_motor_set_velocity(motor_horizontal,-v1);
    end
    wb_motor_set_velocity(motor_horizontal,0);
    %move column up
    while wb_position_sensor_get_value(P1) < INIT_VERTICAL %t<= TIME5
      wb_robot_step(TIME_STEP);
      wb_motor_set_velocity(motor_vertical,v1);
    end
    wb_motor_set_velocity(motor_vertical,0);
    s(1) = s(1)+1;
    if s(1)==4 %the storage is full
        s(2)=0;
    end
  end
end
wb_robot_cleanup();

function t = delay(time)
    t=1;
    while t <= time
      wb_robot_step(1);
      t = t+1;
    end
end



