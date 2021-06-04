import numpy as np
from functools import reduce
from copy import deepcopy

# code for inverse kinematics 
# function returns calculated joint angles in degrees given the end effector coordinates 
INCH_TO_METER = 0.0254
TEST_POSITION_X = 1
TEST_POSITION_Y = 1
TEST_POSITION_PHI =1

def inv_kinematics(x_goal,y_goal,phi_goal_) -> [np.float64, np.float64, np.float64]:
    '''
    Function to derive angles theta1, theta2, theta3
    based on target locations x_goal,y_goal, phi_goal

    Input: x_goal,y_goal,phi_goal
    Output: theta1, theta2, theta3 in degrees
    '''
    phi_goal = np.deg2rad(phi_goal_)

    l1 = 13*INCH_TO_METER   #length of the first segment 
    l2 = 11*INCH_TO_METER   #length of the second segment
    l3 = 4*INCH_TO_METER    #length of the third segment in inches
    b = 10*INCH_TO_METER    #length of the gripper in inches
    l4 = l3+b/2    
    x1 = x_goal - l4*np.cos(phi_goal)
    y1 = y_goal - l4*np.sin(phi_goal)
    A = -2*l1*x1
    B = -2*l1*y1
    alpha = np.arctan2(B/(np.sqrt(A**2+B**2)), A/(np.sqrt(A**2+B**2)))
    #sign = np.array([1,-1])
    theta1 = alpha + np.arccos(-(x1**2+y1**2+l1**2-l2**2)/(2*l1*np.sqrt(x1**2+y1**2)))
    theta2 = np.arccos((x1**2+y1**2-l1**2-l2**2)/(2*l1*l2))
    theta3 = phi_goal-theta1-theta2

    return [np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3)]



def cubic_polynomial(theta_f, t_f, t) -> [np.float64, np.float64, np.float64]:
    '''
    Cubic polynomial trajectory calculation for one joint 
    need to call function for each joint separetely 
    Inputs:  theta_f which is final joint angle (output from inverse kinematics), 
    t_f - final time (total time for joint motion) and 
    t - time step 

    Returns: theta, velocity, acceleration
    '''
    #constraints: 
    theta_0 = 0; # initial position is 0

    #calculated coefficients 
    a0 = theta_0
    a1 = 0
    a2 = (3/t_f**2)*(theta_f - theta_0)
    a3 = (-2/t_f**3)*(theta_f - theta_0)

    # angular position at time step t 
    theta_t = a0 + a1*t + a2*t**2 + a3*t**3

    # angular velocity at time step t 
    angular_vel = a1 + 2*a2*t + 3*a3*t**2

    # angular acceleration at time step t
    angular_acc = 2*a2 + 6*a3*t

    return theta_t,angular_vel,angular_acc

###################### Auxiliary Functions ######################

def Translate(d) -> np.array: 
    '''
    Translate returns translation matrix 
    Returns translation matrix using displacement d 

    Input: Distance d from current point
    Output: Array T
    '''
    # T = np.array([[1,0,0,d], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
    T = np.eye(4) + [[0,0,0,d],[0,0,0,0],[0,0,0,0],[0,0,0,0]]
    return T

def Rotate(theta) -> np.array:
    '''
    Function returns rotation matrix 
    Input angle theta is used to create 3x3 rotation matrix 
    '''
    c = np.cos(theta)
    s = np.sin(theta)
    R = np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])
    return R

###################### Arm Trajectory Generation Code ######################
def arm_trajectory(theta1_,theta2_,theta3_, time, method="velocity"):
    '''
    Forward Kinematics combined with trajectory 
    we want to return angular velocity vector V_arr for 
    joint so we can use it as an input into the joint motors 

    Input: theta1_,theta2_,theta3_,time, method
    Output: 
    '''
    L1 = 13*INCH_TO_METER  #length of the first segment 
    L2 = 11*INCH_TO_METER  #length of the second segment
    L3 = 4*INCH_TO_METER   #length of the third segment 
    b = 10*INCH_TO_METER   #length of the gripper
    L4 = L3+b/2

    theta1 = np.core.umath.deg2rad(theta1_)
    theta2 = np.core.umath.deg2rad(theta2_)
    theta3 = np.core.umath.deg2rad(theta3_)

    # Generate empty lists 
    V_arr = list() #angular velocities for all joints
    A_arr = list() #angular acceleration for all joints
    Angle_arr =list() #angular position of joints
    V1 = list() #angular velocities for joint 1 
    V2 = list() #angular velocities for joint 2
    V3 = list() #angular velocities for joint 3 

    time_arr = np.arange(0,time,0.001)

    for i in time_arr:
        tt1,ang_vel1,ang_acc1 = cubic_polynomial(theta1,time,i)
        tt2,ang_vel2,ang_acc2 = cubic_polynomial(theta2,time,i)
        tt3,ang_vel3,ang_acc3 = cubic_polynomial(theta3,time,i)
    
        ang_vel = [ang_vel1, ang_vel2, ang_vel3]
        ang_acc = [ang_acc1, ang_acc2, ang_acc3]
        angle_arr = [tt1, tt2, tt3]

        A_arr.append(deepcopy(ang_acc))
        Angle_arr.append(deepcopy(angle_arr))

        V1.append(deepcopy(ang_vel[0]))
        V2.append(deepcopy(ang_vel[1]))
        V3.append(deepcopy(ang_vel[2]))
        
    if method=="velocity":
        V1_ = np.array(V1)
        V2_ = np.array(V2)
        V3_ = np.array(V3)
    else:
        V1_ = np.array(V1)
        V2_ = np.array(V2)
        V3_ = np.array(V3)        
    return V1_, V2_, V3_