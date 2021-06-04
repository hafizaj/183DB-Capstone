from controller import *


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os


################################### Constants       ###################################

table_dict = {1:[-0.978356,0.76,-0.591589],
              2:[-0.978356,0.76,0.0084107],
              3:[-0.978356,0.76,0.608411]}


################################### States         ###################################
states ={0:'idle', 1:'moving', 2:'grasping', 3:'terminate'}


################################### Initialization  ###################################


class busserBot(Supervisor):
    def __init__(self):
        super(busserBot, self).__init__()
        self.baseRobot = self.getFromDef('base')
        self.TIME_STEP = 5 # set the control time step


        self.state = 0
        # Get baseRobot devices
        self.lf = self.getDevice('LF')
        self.rf = self.getDevice('RF')

        # Sensors and Tx/Rx devices
        self.emitter = self.getDevice('base_emitter')
        self.receiver = self.getDevice('base_receiver')
        
        self.computer_emitter = self.getDevice('computer_emitter')
        self.computer_receiver = self.getDevice('computer_receiver')
        
        self.connector = self.getDevice('base_connector')

        # Initialize Devices
        self.receiver.enable(self.TIME_STEP)
        self.computer_receiver.enable(self.TIME_STEP)
        self.connector.enablePresence(self.TIME_STEP)
        self.connector.isLocked()

        self.position = self.baseRobot.getPosition()

    def beginOperation(self):
        self.lf.setPosition(float('+inf'))
        self.rf.setPosition(float('+inf'))

        self.lf.setVelocity(0)
        self.rf.setVelocity(0)
        
        
        while (1):
            self.step(self.TIME_STEP)
            self.lf.setVelocity(10)
            self.rf.setVelocity(10)
            if self.receiver.getQueueLength()>0:
                signal = self.receiver.getData()
                
                # the signal from the storage/UI indicates that we can start %moving.
                self.receiver.nextPacket()

                # next, determine the direction we need to move in
                v = 1.0              
                signal_decoded = int.from_bytes(signal, byteorder='big')
                table_coordinates = table_dict[signal_decoded]
                
                # Once we have table coordinates, then we can move to the location
                # In 1-D, we only need the either x or z
                # In this case, we use z
                while(self.position[2] != table_coordinates[2]):
                    self.step(self.TIME_STEP)
                    self.position = self.baseRobot.getPosition() 
                    if (self.position[2] < table_coordinates[2]):
                        speed = v
                    else:
                        speed = -v
                    self.lf.setVelocity(speed)
                    self.rf.setVelocity(speed)
                    
                self.lf.setVelocity(0)
                self.rf.setVelocity(0)
                print('test')
            # Send command to start picking up the tray
            elif self.computer_receiver.getQueueLength()>0:
                signal_computer = self.computer_receiver.getData()
                self.computer_receiver.nextPacket()

                # next, determine the direction we need to move in
                v = 1.0              
                signal_comp_decoded = int.from_bytes(signal_computer, byteorder='big')
                table_coordinates_computer = table_dict[signal_comp_decoded]
                while(self.position[2] != table_coordinates_computer[2]):
                    self.step(self.TIME_STEP)
                    self.position = self.baseRobot.getPosition() 
                    if (self.position[2] < table_coordinates_computer[2]):
                        speed = v
                    else:
                        speed = -v
                    self.lf.setVelocity(speed)
                    self.rf.setVelocity(speed)


            self.emitter.send(bytes([1]))

bbot = busserBot()
bbot.beginOperation()

################################### Initialization  ###################################

