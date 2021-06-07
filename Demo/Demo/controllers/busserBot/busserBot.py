from controller import *
# from cont import *

from cont import *


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

os.environ['PATH']


################################### Constants       ###################################
kitchen_coordinate = np.array([-1.0,0.0,-7.0])

table_dict = {1: [-4.25,0.763,-3],
         2:[-4.25,0.763,-1],
         3:[-4.25,0.763,1],
         4:[-4.25,0.763,3]}
table_dict = {1: [-4.25,0.763,-6],
         2:[-4.25,0.763,-4],
         3:[-4.25,0.763,-2],
         4:[-4.25,0.763,0]}
THRESHOLD = 0.0001
################################### States         ###################################
states ={0:'idle', 1:'moving', 2:'grasping', 3:'terminate'}
# Once we begin grasping, then we activate the arm nontheless

################################### Initialization  ###################################


class busserBot(Supervisor):
    def __init__(self):
        super(busserBot, self).__init__()
        self.baseRobot = self.getFromDef('base')
        self.timeStep = 5 # set the control time step

        # Robot states and table queues
        self.state = 0
        self.table_queue = list()
        
        # Get baseRobot devices
        self.slider_joint = self.getDevice('slider_joint')
        self.slider_sensor = self.getDevice('slider_sensor')
        self.armController = ArmController(self)

        # Sensors and Tx/Rx devices
        # Tx/Rx to and from the terminal
        self.computer_emitter = self.getDevice('computer_emitter')
        self.computer_receiver = self.getDevice('computer_receiver')
        
        # Initialize Devices
        # Initialize the device of the robot
        self.computer_receiver.enable(self.timeStep)
        self.slider_sensor.enable(self.timeStep)
        
        # Detect the original position of the robot
        self.position = self.slider_sensor.getValue()
 
    ################################### Initialization  ###################################
    
    # def initializeMotor(self):
    #     self.lf.setPosition(float('+inf'))
    #     self.rf.setPosition(float('+inf'))

    #     self.lf.setVelocity(0)
    #     self.rf.setVelocity(0)
        
    def func_idle(self):
        # Get commands from the terminal
        if (self.checkReceiverStatus() and 
            (self.armController.checkIfStorageIsFull()==False)):
                self.getTableCoordinates()
                
        if len(self.table_queue)>0:
            # print('Receving commands...')
            self.position = self.slider_sensor.getValue()
            self.state = 1
        else:
            # print('Awaiting commands...')
            self.position = self.slider_sensor.getValue()
            pass
            
    def func_moving(self):
        print('Moving...')
        # Move according to table coordinate
        current_table_coordinate = self.table_queue.pop()
        while(np.abs(self.position - current_table_coordinate[2])>THRESHOLD):
            self.step(self.timeStep)
            self.position = self.slider_sensor.getValue()
            self.slider_joint.setPosition(current_table_coordinate[2])
        print('Finished Moving')
        # self.computer_emitter.send(bytes([1]))
        self.state = 2
        
    def func_grasping(self):
        # Grasp object
        print('Grasping...')
        self.armController.collectTray(0,0,0,6)
        # Check if the storage is full and there are remaining 
        # tables in the queue
        if self.armController.checkIfStorageIsFull() == False:
            print('Storage allows for more collection')
            # If there are no tables left in the queue, we just pause
            if len(self.table_queue)>0: 
                self.state = 1
            else:
                self.state = 0
                
        else:
            self.state = 3
        
        # if len(self.table_queue)>0: 
        #     self.state = 1
        # else:
        #     self.state = 0
    
    def func_exit(self):
        print('Exiting')
        while(np.abs(self.position - kitchen_coordinate[2])>THRESHOLD):
            self.step(self.timeStep)
            self.position = self.slider_sensor.getValue()
            self.slider_joint.setPosition(kitchen_coordinate[2])
        self.computer_emitter.send(bytes([2]))
        self.state = 0

    def checkState(self):
        '''
        Checks the state of the robot on each iteration
        State 0 : Do nothing
        State 1 : Move to the destination and verify that it reaches the destination
        State 2 : Move the arm and begin the storing process
        State 3 : End the motion of the robot

        and executes the function corresponding to the library
        '''
        stateFunctions.get(self.state, lambda: 'Invalid')(self)

    ################################### Operations  ###################################
    def checkReceiverStatus(self) -> bool:
        '''
        Check if there is anything in the receiver
        Return true if there is signal in the receiver
        Return false if there is nothing in the receiver
        '''
        return (self.computer_receiver.getQueueLength()>0)
    
    def checkStorageStatus(self) -> bool:
        '''
        Checks if the storage is full or not
        Interfaces with the storage unit to verify the state of the 
        '''
    
    def getTableCoordinates(self):
        '''
        Add all the queued tables into the table_queue
        '''
        # the signal from the storage/UI indicates that we can start moving.
        while (self.checkReceiverStatus()):
            signal = self.computer_receiver.getData()
            self.computer_receiver.nextPacket()
            signal_decoded = int.from_bytes(signal, byteorder='big')
            table_coordinates = table_dict[signal_decoded]
            self.table_queue.append(table_coordinates)

    def beginOperation(self, global_runtime=6):
        self.runtime = global_runtime
        
        while (1):
            self.step(self.timeStep)
            # If we have something from the terminal, then add to the queue
            if (self.checkReceiverStatus()):
                self.getTableCoordinates()
            # If we have something in the queue, iterate on the item   
            # print(self.state)     
            self.checkState()
            
    def measureMotion(self):
        bbot.armController.collectTray(0,0,0,6)

################################### Dictionary  ###################################
stateFunctions = {0: busserBot.func_idle, 
                  1: busserBot.func_moving, 
                  2: busserBot.func_grasping, 
                  3: busserBot.func_exit}

################################### Running  ###################################

bbot = busserBot()
# bbot.armController.collectTray(0,0,0,6)
bbot.beginOperation()
