import numpy as np
import csv
from copy import deepcopy
from functools import reduce

# Parameter values
L1_VALUE = 0.3302
L2_VALUE = 0.2794
L3_VALUE = 0.1016
L4_VALUE = 0.2286

class wayPoints:
    def __init__(self,filename = None):
        self.waypoints = []
        if filename != None:
            self.readFromFile(filename)
        
    def addWP(self,wp):
        #wp is (j1,j2,jn,t) tuple
        self.waypoints.append(wp)
        
    def readFromFile(self,filename):
        #assuming CSV file
        with open(filename,newline='') as wpFile:
            wpReader = csv.reader(wpFile, delimiter=',', quotechar='|')
            for wp in wpReader:
                newRow = [float(val) for val in wp]
                self.addWP(tuple(newRow))

class trajectoryPlanner:
    def __init__(self, jointNum):
        self.jointNum = jointNum
        self.method = None
        
    def waypointsParse(self,wayP,method):
        #interpret a list of waypoints
        #list of tuples, [(joint1_i,joint2_i,...,t_i)...]
        #interpolate between points with method
        self.method = method
        
        if len(wayP) < 2:
            print("not enough waypoints!")
            return
        
        #check for positive time increments
        tLast = 0
        for waypoint in wayP:
            tDiff = waypoint[-1]-tLast
            tLast = waypoint[-1]
            if tDiff < 0:
                print("not a positive time sequence!")
                return
        
        if len(wayP[0])-1 != self.jointNum:
            print("waypoints/joint mismatch!")
            return
        
        self.joints = []
        for i in range(self.jointNum):
            self.joints.append([])
        
        #split waypoints into individual [(position,time)] per joint
        for waypoint in wayP:
            for i in range(self.jointNum):
                self.joints[i].append([waypoint[i],waypoint[-1]])

        #joints[] is now a list of points,time per joint
        if method == "cubic":
            #cubic interpolation
            self.cubicInterpolate()
        elif method == "quintic":
            #quintic interpolation
            self.quinticInterpolate()
        elif method == "trapVel":
            self.trapVelInterpolate()
        else:
            pass
            
    def trapVelInterpolate(self):
        #interpolate between points with trapezoidal velocity
        #later - respect max velocity and acceleration constraints
        #no blending - stop at waypoints.
        pass
        
            
    def cubicInterpolate(self):
        #for each joint, calculate the coefficients for the cubic
        #interpolating polynomials
        
        #modify joints from [position,time] to [position,velocity,time]
        self.cubicCoeffs = []
        for j in self.joints:
            self.cubicCoeffs.append([])
        
        #null starting and ending velocity
        for j in self.joints:
            for i in range(len(j)):
                j[i].insert(1,0)
            
        #compute intermediate velocities
        #if the last and next velocities are of different sign, make 
        #velocity = 0 at that point.    
        for j in self.joints:
            for i in range(1,len(j)-1):
                #(pos-lastPos)/(tdiffLast)
                velBefore = (j[i][0]-j[i-1][0])/(j[i][-1]-j[i-1][-1])
                #(nextPos-pos)/(tdiffNext)
                velAfter = (j[i+1][0]-j[i][0])/(j[i+1][-1]-j[i][-1])
                if sign(velBefore) == sign(velAfter):
                    intermediateVelocity = (1/2)*(velBefore+velAfter)
                else:
                    # intermediateVelocity = 0
                    intermediateVelocity = (1/2)*(velBefore+velAfter)
                j[i][1] = intermediateVelocity
        
        #now, do the interpolation
        #treat each points starting time as 0, simpler math
        for num,j in enumerate(self.joints):
            for i in range(len(j)-1):
                #calculate a3...a0 for the cubic polynomial between
                #each pair of points
                
                qi = j[i][0]
                qi_dot = j[i][1]
                qf = j[i+1][0]
                qf_dot = j[i+1][1]
                tf = j[i+1][-1]-j[i][-1]
                
                A = np.array([[1,0,0,0],[0,1,0,0],[1,tf,tf**2,tf**3],[0,1,2*tf,3*tf**2]])
                B = np.array([[qi],[qi_dot],[qf],[qf_dot]])
                coeffs = np.linalg.solve(A,B)
                coeffs = np.append(coeffs,tf)
                self.cubicCoeffs[num].append(coeffs)
                
                
        
    def quinticInterpolate(self):
        #same as cubic but with continuous accelerations
        
        #modify joints from [position,time] to [position,velocity,accel,time]
        self.quinticCoeffs = []
        for j in self.joints:
            self.quinticCoeffs.append([])
        
        #null starting and ending velocity and acceleration
        for j in self.joints:
            for i in range(len(j)):
                j[i].insert(1,0)
                j[i].insert(1,0)
            
        #compute intermediate velocities
        #if the last and next velocities are of different sign, make 
        #velocity = 0 at that point.    
        for j in self.joints:
            for i in range(1,len(j)-1):
                #(pos-lastPos)/(tdiffLast)
                velBefore = (j[i][0]-j[i-1][0])/(j[i][-1]-j[i-1][-1])
                #(nextPos-pos)/(tdiffNext)
                velAfter = (j[i+1][0]-j[i][0])/(j[i+1][-1]-j[i][-1])
                if sign(velBefore) == sign(velAfter):
                    intermediateVelocity = (1/2)*(velBefore+velAfter)
                else:
                    intermediateVelocity = 0
                j[i][1] = intermediateVelocity
                
        #compute intermediate accelerations
        #if the last an next accel are diff sign, make accel = 0
        #at midpoint
        for j in self.joints:
            for i in range(1,len(j)-1):
                #(vel-lastVel)/(tdiffLast)
                accelBefore = (j[i][1]-j[i-1][1])/(j[i][-1]-j[i-1][-1])
                #(nextVel-vel)/(tdiffNext)
                accelAfter = (j[i+1][1]-j[i][1])/(j[i+1][-1]-j[i][-1])
                if sign(accelBefore) == sign(accelAfter):
                    intermediateAccel = (1/2)*(accelBefore+accelAfter)
                else:
                    intermediateAccel = 0
                j[i][2] = intermediateAccel
        
        #now, do the interpolation
        #treat each points starting time as 0, simpler math
        for num,j in enumerate(self.joints):
            for i in range(len(j)-1):
                #calculate a3...a0 for the cubic polynomial between
                #each pair of points
                
                qi = j[i][0]
                qi_dot = j[i][1]
                qi_ddot = j[i][2]
                qf = j[i+1][0]
                qf_dot = j[i+1][1]
                qf_ddot = j[i+1][2]
                tf = j[i+1][-1]-j[i][-1]
                
                A = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,2,0,0,0],[1,tf,tf**2,tf**3,tf**4,tf**5],[0,1,2*tf,3*tf**2,4*tf**3,5*tf**4],[0,0,2,6*tf,12*tf**2,20*tf**3]])
                B = np.array([[qi],[qi_dot],[qi_ddot],[qf],[qf_dot],[qf_ddot]])
                coeffs = np.linalg.solve(A,B)
                coeffs = np.append(coeffs,tf)
                self.quinticCoeffs[num].append(coeffs)
        
    def calcOutputs(self,Ts):
        
        #calculate p,v,a outputs for each joint
        #sample polynomials with time interval Ts
        
        #self.outputs is a list, [[[j1_pi,j1_vi,j1_ai],...]...]
        self.outputs = []
        for j in self.joints:
            self.outputs.append([])
            
        if self.method == None:
            print("Paths have not been interpolated")
            return
        elif self.method == "cubic":
            i = 0
            for joint in self.cubicCoeffs:
                for j, c in enumerate(joint):
                    if j == 0:
                        #create 1st value
                        _t = 0
                        p = c[0] + c[1]*_t + c[2]*_t**2 + c[3]*_t**3
                        v = c[1] + 2*c[2]*_t + 3*c[3]*_t**2
                        a = 2*c[2] + 6*c[3]*_t
                        self.outputs[i].append([p,v,a])
                        
                    for _t in np.arange(Ts,c[-1]+Ts,Ts):
                        p = c[0] + c[1]*_t + c[2]*_t**2 + c[3]*_t**3
                        v = c[1] + 2*c[2]*_t + 3*c[3]*_t**2
                        a = 2*c[2] + 6*c[3]*_t
                        self.outputs[i].append([p,v,a])
                        
                i = i + 1
        elif self.method == "quintic":
            i = 0
            for joint in self.quinticCoeffs:
                for j, c in enumerate(joint):
                    if j == 0:
                        #create 1st value
                        _t = 0
                        p = c[0] + c[1]*_t + c[2]*_t**2 + c[3]*_t**3 + c[4]*_t**4 + c[5]*_t**5
                        v = c[1] + 2*c[2]*_t + 3*c[3]*_t**2 + 4*c[4]*_t**3 + 5*c[5]*_t**4
                        a = 2*c[2] + 6*c[3]*_t + 12*c[4]*_t**2 + 20*c[5]*_t**3
                        self.outputs[i].append([p,v,a])
                        
                    for _t in np.arange(Ts,c[-1]+Ts,Ts):
                        p = c[0] + c[1]*_t + c[2]*_t**2 + c[3]*_t**3 + c[4]*_t**4 + c[5]*_t**5
                        v = c[1] + 2*c[2]*_t + 3*c[3]*_t**2 + 4*c[4]*_t**3 + 5*c[5]*_t**4
                        a = 2*c[2] + 6*c[3]*_t + 12*c[4]*_t**2 + 20*c[5]*_t**3
                        self.outputs[i].append([p,v,a])
                        
                i = i + 1
        elif self.method == "trapVel":
            pass
        else:
            return

###################### Required Functions ######################

       
def sign(num):
    if num > 0:
        return 1
    elif num < 0:
        return -1
    else:
        return 0

def speedChange(waypoints, multiplier):
    newPoints = []
    for i in range(len(waypoints)):
        newPoints.append((waypoints[i][0],waypoints[i][1],waypoints[i][2]*multiplier))
    return newPoints
    
def generateWaypoints(theta1_, theta2_, theta3_, time, theta_sampling_rate=0.003):
    theta1 = np.deg2rad(theta1_)
    theta2 = np.deg2rad(theta2_)
    theta3 = np.deg2rad(theta3_)
    #Product of all 3 angles will be a multiple of LCM of 3 angles
    theta = theta1 * theta2 * theta3

    if theta < 0:
        theta = -theta


    theta_iterator = np.arange(0,theta,theta_sampling_rate)

    waypoints_arr = list()

    time_arr = np.linspace(0, time,theta_iterator.size)
    j = 0
    for i in theta_iterator:
        tt1 = (i* theta1) / theta
        tt2 = (i* theta2) / theta
        tt3 = (i* theta3) / theta
        tuple_saved =  deepcopy( (tt1, tt2, tt3, time_arr[j]) )
        waypoints_arr.append( tuple_saved )
        j +=1
    return waypoints_arr
    
def generateWaypointModified(theta1, theta2, theta3, tf, iteration_steps=0.001):
    '''
    Takes in theta1, theta2, theta3
    returns waypoints

    Input: theta1, theta2, theta3, tf, iteration_steps
    Output: waypoints p1, p2, p3
    '''    
    T = np.arange(0, tf, iteration_steps)
    waypoints_arr = list()

    for i in T:
        pos1,_,_ = cubic_polynomial(np.deg2rad(theta1),tf,i)
        pos2,_,_ = cubic_polynomial(np.deg2rad(theta2),tf,i)
        pos3,_,_ = cubic_polynomial(np.deg2rad(theta3),tf,i)
        waypoints_arr.append((pos1, pos2, pos3, i) )

    return waypoints_arr       
    
###################### Kinematic Modelling Functions ######################

def forward_kinematics(theta1,theta2,theta3) -> np.array:
    '''
    Takes in angle theta1, theta2, theta3
    and returns the coordinates x,y, and phi

    Input: theta1, theta2, theta2
    Output: np.array of outputs [x,y,phi]
    '''
    # Link Parameters
    L1 = L1_VALUE  #length of the first segment 
    L2 = L2_VALUE  #length of the second segment
    L4 = L4_VALUE

    x = np.array([0, L1, L1+L2, L4+L1+L2]);
    y = np.array([0., 0., 0., 0.]);

    # Convert to radians
    tt1 = np.deg2rad(theta1)
    tt2 = np.deg2rad(theta2)
    tt3 = np.deg2rad(theta3)

    # Rotation Matrices
    R1 = Rotate(tt1)
    R2 = Rotate(tt2)
    R3 = Rotate(tt3)

    # Translation Matrices
    # Parameters are link lengths
    T2 = Translate(L1)
    T3 = Translate(L2)
    T4 = Translate(L4)

    # First joint position
    # Transformation matrix

    a = np.array([[0],[0],[0],[1]])
    Y_iter1 = reduce(np.dot, [R1,T2])
    # Finding new coordinates
    Y1 = np.dot(Y_iter1, a)

    x[1] = Y1[0]
    y[1] = Y1[1]

    # Finding third point
    # transformation matrix
    Y_iter2 = reduce(np.dot, [R1,T2,R2,T3])
    # Finding new coordinates
    Y2 = np.dot(Y_iter2, a)
    x[2] = Y2[0]
    y[2] = Y2[1]

    # finding fourth point

    # transformation matrix
    Y_iter3 = reduce(np.dot, [R1,T2,R2,T3,R3,T4])

    # Finding new coordinates
    Y3 = np.dot(Y_iter3, a)

    x[3] = Y3[0]
    y[3] = Y3[1]

    phi = np.rad2deg(tt1+tt2+tt3)
    return x,y,phi

    
def inv_kinematics(x_goal,y_goal,phi_goal_) -> [np.float64, np.float64, np.float64]:
    '''
    Function to derive angles theta1, theta2, theta3
    based on target locations x_goal,y_goal, phi_goal

    Input: x_goal,y_goal,phi_goal
    Output: theta1, theta2, theta3 in degrees
    '''
    phi_goal = np.deg2rad(phi_goal_)
    l1 = L1_VALUE   #length of the first segment 
    l2 = L2_VALUE   #length of the second segment
    l4 = L4_VALUE   
     
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
    
def cubic_polynomial(theta_f, t_f, t):
    #constraints: 
    theta_0 = 0; # initial position is 0
    #dot_theta_0 = 0; %initial velocity is zero 
    #dot_theta_f = 0 %final velocity is zero 

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