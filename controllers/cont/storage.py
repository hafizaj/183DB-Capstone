# MATLAB CONTROLLER CODE
# Created by Ahmad Hafizuddin bin Ahmad Jaafar

from controller import *
import numpy as np


# Required parameters
DISTANCE_FROM_VERTICAL_PLATFORM_TO_TOP_FLOOR = 0.23
DISTANCE_FROM_VERTICAL_PLATFORM_TO_BOT_FLOOR = 0.47
DISTANCE_FROM_INITIAL_HORIZONTAL_POS_TO_LEFT_COL = 0.58
DISTANCE_FROM_INITIAL_HORIZONTAL_POS_TO_RIGHT_COL = 0.27

in_s = [0, 1]
# in_s = [0, 0]
######################## Auxiliary Controller ########################

timeStep= 5
def move_motor(position, v1, v2, v3):
    '''
    Position is vertical,horizontal
    '''
    position_map = {1:[0.25,0.62], 2:[0.25,0.37], 3:[0.5,0.62], 4:[0.5,0.37]}
    speedg = v1
    speedh = v2
    speedv = v3
    vert_dist, hori_dist = position_map[position]
    # Find the time for vertical column to go down
    TIME1 = np.ceil(1000*(vert_dist - 0.02)/speedg) #in ms
    # Find the time for column to go left
    TIME2 = np.ceil(1000*hori_dist/speedh)
    # Find the time for column to drop the tray into location
    TIME3 = float(np.ceil(1000*0.04/speedv)) / float(timeStep) 
    # find the time for column to go to the left
    TIME4 = np.ceil(1000*hori_dist/speedg)
    # Find the time for column to go back to initial position
    TIME5 = np.ceil(1000*(vert_dist + 0.02)/speedg)
    return TIME1, TIME2, TIME3, TIME4, TIME5

######################## Storage Controller ########################
class StorageController():
    def __init__(self, RobotMain):
        '''
                    Define the operations to be as follows
                    '''
        self.RobotMain = RobotMain
        self.state = 0
        
        self.motor_horizontal = self.RobotMain.getDevice('horizontal')
        self.motor_vertical = self.RobotMain.getDevice('vertical')
        # self.arm_vertical = self.RobotMain.getDevice('arm_vertical')

        self.a0 = self.RobotMain.getDevice('a0')
        self.P1 = self.RobotMain.getDevice('P1')
        self.P2 = self.RobotMain.getDevice('P2')

        # Initial States
        self.a = [0, 0]
        self.s = in_s

    def enableSensors(self):
        '''
        Enables sensing devices
        '''
        # Enable devices
        # Touch Sensor
        self.a0.enable(self.RobotMain.timeStep)
        # Position Sensors
        self.P1.enable(self.RobotMain.timeStep)
        self.P2.enable(self.RobotMain.timeStep)

    def storeObject(self, duration=1):
        '''
        Simulate storing motion within prescribed duration
        Input: duration (float)
        Output: void
        '''
        timeStepConstant = self.RobotMain.timeStep

        self.RobotMain.step(timeStepConstant)
        # Motion location
        INIT_VERTICAL = self.P1.getValue()
        INIT_HORIZONTAL = self.P2.getValue()
        TOP_FLOOR = INIT_VERTICAL - 0.23
        BOTTOM_FLOOR = INIT_VERTICAL - 0.47
        LEFT_CLMN = INIT_HORIZONTAL + 0.58
        RIGHT_CLMN = INIT_HORIZONTAL + 0.27

        
        while (self.RobotMain.step(timeStepConstant)!=-1):
            self.a[0] = self.a0.getValue()
            if self.a[1]==1:
                #Clear the storage unit
                self.s[0]=0
            elif (self.a[0]==0 or self.s[1]==0):
                # Full or defer
                pass
            elif (self.a[0]==1):
                # Detect something, then
                if (self.s[0] == 0 or self.s[0] == 1):
                    vert = TOP_FLOOR
                else:
                    vert = BOTTOM_FLOOR

                if (self.s[0] == 0 or self.s[0]==2):
                    hori = LEFT_CLMN
                else:
                    hori = RIGHT_CLMN

                self.delay(500)

                v1 = 0.2 # going down first time, going right, and up
                v2 = 0.1 # going to the left
                v3 = 0.1 # placing the tray
                print('test')
                TIME1,TIME2,TIME3,TIME4,TIME5 = move_motor(self.s[0]+1,v1,v2,v3)

                # Move column down
                while self.P1.getValue()> vert:
                    self.RobotMain.step(timeStepConstant)
                    self.motor_vertical.setPosition(float('+inf'))
                    self.motor_vertical.setVelocity(-v1)

                self.motor_vertical.setVelocity(0)
                self.delay(500)

                # Move column to the left
                while (self.P2.getValue() < hori):
                    self.RobotMain.step(timeStepConstant)
                    self.motor_horizontal.setPosition(float('+inf'))
                    self.motor_horizontal.setVelocity(v2)
                self.motor_horizontal.setVelocity(0)
                self.delay(500)

                # Move column down
                t = 0
                while t < TIME3:
                    self.RobotMain.step(timeStepConstant)
                    self.motor_vertical.setVelocity(-v3)
                    t = t+1
                self.motor_vertical.setVelocity(0)
                # Move column right
                while (self.P2.getValue() > INIT_HORIZONTAL):
                    self.RobotMain.step(timeStepConstant)
                    self.motor_horizontal.setVelocity(-v1)
                self.motor_horizontal.setVelocity(0)

                # Move column up
                while (self.P1.getValue() < INIT_VERTICAL):
                    self.RobotMain.step(timeStepConstant)
                    self.motor_vertical.setVelocity(v1)
                self.motor_vertical.setVelocity(0)

                # Update State
                self.s[0] = self.s[0]+1
                if self.s[0]==4 :
                    self.s[1]=0
                
    def delay(self, time):
        target_time = time/float(self.RobotMain.timeStep)
        t=0
        while t < time:
          self.RobotMain.step(self.RobotMain.timeStep)
          t = t+1
        return t



        
