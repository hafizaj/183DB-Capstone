from controller import *
import numpy as np

GRIPPER_STATES = {0:'off', 1:'contract', 2:'grasp', 3:'release'}
TRAY_LOCATION = None
LIFTING_COLUMN_LOCATION = None
LEFT_POSITION_SETTING = 0
RIGHT_POSITION_SETTING = 0


class GripperController():
    def __init__(self, RobotMain):
        '''
    Define the states to be as follows
    off - gripper is in inactive state
    contract - gripper is contracting to grasp the tray by the sides
    grasp - gripper is in grasping configuration and will
    release - 
    '''
        self.RobotMain = RobotMain
        self.state = 0
        self.gripperLeftMotor = self.RobotMain.getDevice('leftg')
        self.gripperRightMotor = self.RobotMain.getDevice('rightg')
        self.leftPosition = self.RobotMain.getDevice('leftg_sensor')
        self.rightPosition = self.RobotMain.getDevice('right_gsensor')


    def contract(self):
        '''
        Constricts the object upon reaching the target location (Tray)
        Enable connector node here
        '''
        print('Begin Gripping')
        while(self.rightPosition.getValue() != LEFT_POSITION_SETTING or self.rightPosition.getValue()  != RIGHT_POSITION_SETTING):
            self.RobotMain.step(self.RobotMain.timeStep)
            self.gripperLeftMotor.setPosition(LEFT_POSITION_SETTING)
            self.gripperRightMotor.setPosition(RIGHT_POSITION_SETTING)
        print('Object Gripped')
        # If the right is closer to the left, then connect to the left
        self.RobotMain.step(self.RobotMain.timeStep)
        self.RobotMain.frontConnector.lock()    

    def release(self):
        '''
        Releases the gripper upon reaching the target location (Lifting Column)
        Takes in parameters
        Disable connector node here
        '''
        print('Release Object')
        while(self.rightPosition.getValue() != 0 or self.rightPosition.getValue()  != 0):
            self.RobotMain.step(self.RobotMain.timeStep)
            self.gripperLeftMotor.setPosition(0)
            self.gripperRightMotor.setPosition(0)
        print('Object Released')
        # If the right is closer to the left, then connect to the left
        self.RobotMain.step(self.RobotMain.timeStep)
        self.RobotMain.frontConnector.unlock() 
        self.RobotMain.step(self.RobotMain.timeStep)

    def grasp(self):
        '''
        Pobot is grasping the object
        Maintain the force applied
        Verify that connectors are still working
        '''
        pass

    def run(self):
        '''
        Checks the current state of the robot and perform commands
        '''
        pass