from controller import *
from t_gen import *
from storage import *
from gripper import *


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os


NUM_DOF = 3

# Slider Parameters
GROUND_OFFSET = 0.21225
VERTICAL_SLIDER_POSITION = -0.35 -GROUND_OFFSET
SLIDER_THRESHOLD = 0.0001
PLACING_OFFSET = 0.053

######################## Auxiliary Functions  ########################

def saveFileName(df_obj, filename, dir='Data'):
    '''
    Saves csv to file using `filename` in `directory` provided
    
    Input: 
    df_obj - pandas dataFrame object
    filename - string
    dir - string
    
    Output:
    None
    '''
    # Get current directory
    curr_dir = os.getcwd()
    os.makedirs('new_directory', exist_ok=True)
    
    working_directory = os.path.join(curr_dir, dir)
    print(working_directory)
    save_name = f"{working_directory}/{filename}.csv"
    df_obj.to_csv(save_name)

######################## Arm Controller ########################
class ArmController():
    def __init__(self, RobotMain):
        self.RobotMain = RobotMain
        super(ArmController, self).__init__()
        self.timeStep = self.RobotMain.timeStep  # set the control time step
        self.save_trajectories = False

        # Initialize Motor and set position
        shoulderMotor = self.RobotMain.getDevice('ShoulderHingeMotor')
        elbowMotor = self.RobotMain.getDevice('ElbowMotor')
        wristMotor = self.RobotMain.getDevice('WristMotor')
        self.motors = [shoulderMotor, elbowMotor, wristMotor]
        
        # Initialize and enable device tags
        self.dsEndEffector = self.RobotMain.getDevice('dsEndEffector')
        self.endEffectorLocation = self.RobotMain.getDevice('endEffectorLocation')
        self.ShoulderSensor = self.RobotMain.getDevice('ShoulderSensor')
        self.ElbowSensor = self.RobotMain.getDevice('ElbowSensor')
        self.WristSensor = self.RobotMain.getDevice('WristSensor')
        
        # Track Completion
        # Represents the targeted motion of the joints
        self.completedMotion = [False, False, False]
        self.objectDetected = False
        
        # Storage Unit
        self.storageCont = StorageController(self.RobotMain)
        
        # End Effector Trajectory
        # Connector Devices
        self.gripperTool = GripperController(self.RobotMain)
        self.frontConnector = self.RobotMain.getDevice('frontConnector')
        self.frontConnector.enablePresence(self.timeStep)
        
        # Vertical Slider
        self.verticalSlider = self.RobotMain.getDevice('verticalSlider')
        self.verticalSliderPosition = self.RobotMain.getDevice('verticalSliderPosition')
    ###################### Motor Control ######################
    def setMotorMode(self, mode="position"):
        self.mode = mode

    def motorMode(self):
        '''
        Checks the mode of operation of the motor
        If mode is set to "position", then it will not set motor position to infinity
        Otherwise, motor position will be set to infinity 
        '''
        if self.mode == "position":
            pass
        else:        
            for m in self.motors:
                m.setPosition(float('+inf'))


    def moveMotor(self, theta1, theta2, theta3):
        '''
        Takes in 3 angles theta1, theta2, theta3
        and moves arm towards the direction
        '''
        if self.mode == "position":
            '''
            Position control:
            Motor movement is defined by the PID control
            '''
            theta_arr = [theta1, theta2, theta3]
            for i in range(len(self.motors)):
                m.setPosition(theta_arr[i])

        else:
            pass
    
    def moveVerticalSlider(self, target_position, threshold= SLIDER_THRESHOLD):
        '''
        Takes in a position to move the vertical slider
        '''
        while(np.abs(self.verticalSliderPosition.getValue()- target_position)>threshold):
            self.RobotMain.step(self.timeStep)
            self.verticalSlider.setPosition(target_position)
 
    def stopMotors(self):
        for i in range(NUM_DOF):
            # print(V[j][i])
            self.motors[i].setVelocity(0)
    
    ###################### Auxiliary Functions ######################
    def checkCurrentAngles(self):
        '''
        Returns the current position of the motors
        '''
        currTheta1 = self.ShoulderSensor.getValue()
        currTheta2 = self.ElbowSensor.getValue()
        currTheta3 = self.WristSensor.getValue()
        
        return np.array([currTheta1, currTheta2, currTheta3])
        
    def isThereObject(self, sensorReading):
        '''
        Detects whether there is an object in front of the sensor
        '''
        if sensorReading > 500:
            self.objectDetected = True
        else:
            self.objectDetected = False
            
    def determinePosition(self):
        '''
        Returns the current position
        '''
        li = list()
        li.append(self.ShoulderSensor.getValue())
        li.append(self.ElbowSensor.getValue())
        li.append(self.WristSensor.getValue())
        li.append(0)
        li_ret = tuple(li)

        return li_ret

    ######################  Storage Analysis ######################
    def checkIfStorageIsFull(self) -> bool:
        '''
        Check if the storage unit is full or not
        '''
        return (self.storageCont.is_full)

    def testStorage(self):
        self.storageCont.enableSensors()
        self.storageCont.storeObject()
        
    def testCamera(self):
        self.shoulderCamera = self.getDevice('shoulderCamera')
        self.shoulderCamera.recognitionEnable(2*self.timeStep)
        self.shoulderCamera.enable(2*self.timeStep)

    ######################  Initialize Functions ######################

    def initializeSensors(self):
        # Enable position sensors
        self.ShoulderSensor.enable(self.timeStep)
        self.WristSensor.enable(self.timeStep)
        self.ElbowSensor.enable(self.timeStep)
        self.storageCont.enableSensors()
        
        # Enable direct sensors
        self.dsEndEffector.enable(self.timeStep)
        self.endEffectorLocation.enable(self.timeStep)        

    ######################  Motion Analysis ######################
    def actualTrajectories(self, theta1,theta2,theta3, time, mode='quintic'):
        '''
        Returns the numerical solutions for trajectories
        '''
        Ts = 0.001
        waypoints = [self.takeInitialPosition(),(theta1,theta2,theta3,time)]
        
        # Appending last time element
        time_arr = np.arange(0,time, Ts)
        test = time_arr.tolist()
        test.append(time)
        time_arr = np.array(test)

        joint = trajectoryPlanner(3)
        joint.waypointsParse(waypoints, mode)

        joint.calcOutputs(Ts) 
        jointOutputs = np.array([joint.outputs[0], joint.outputs[1], joint.outputs[2]])
        i=1
        for j in jointOutputs:
            position_data = np.array(j[:,0])
            velocity_data = np.array(j[:,1])
            acceleration_data = np.array(j[:,2])
            outputData = np.column_stack((time_arr,position_data,velocity_data,acceleration_data ))
            filename = f'joint_{i}.csv'
            np.savetxt(filename, outputData, delimiter=',')
            i+=1

        theta_1 = np.array(jointOutputs[0][:,0])
        theta_2 = np.array(jointOutputs[1][:,0])
        theta_3 = np.array(jointOutputs[2][:,0])
        theta_array = np.vstack((theta_1.T,theta_2.T, theta_3.T)) 
        results_ = list()
        for i in range(theta_1.size):
            x_, y_, phi_ = forward_kinematics(theta_array[0][i], theta_array[1][i], theta_array[2][i])
            output_list = [x_[3], y_[3], phi_]
            # print(output_list)
            results_.append(output_list)
        results = np.array(results_)
        filenameResults = 'end_effector_mapped_points.csv'
        np.savetxt(filenameResults, results, delimiter=',')
        # Map to end-effector space 

    ###################### Beginning Motion  ######################
    def setRunTime(self, t_f=5.0):
        self.runTime = t_f

    def generateVelocities(self, theta1_, theta2_, theta3_, 
                           runtime_,operation_name='default'):
        '''
        Returns the velocity vectors for given angles
        '''
        theta1= np.deg2rad(theta1_)
        theta2= np.deg2rad(theta2_)
        theta3= np.deg2rad(theta3_)
        
        # Initialize settings
        self.setRunTime(runtime_)
        self.setMotorMode("velocity")
        self.motorMode()

        traj_li = list([])
        joint_li = list([])

        self.RobotMain.step(self.timeStep)
        
        waypoints = [self.determinePosition(),(theta1,theta2,theta3,runtime_)]
        
        
        tp = trajectoryPlanner(3)
        tp.waypointsParse(waypoints, "quintic")
        tp = trajectoryPlanner(3)
        tp.waypointsParse(waypoints, "quintic")
        # Calculates p, v, a
        tp.calcOutputs(self.timeStep/1000)

        vectorSizes = len(tp.outputs[1])
        self.stopMotors()
        for i in range(vectorSizes):
            # i is the time instance
            # j would be the indexing of each motor joint
            self.RobotMain.step(self.timeStep)
            
            # If motor is not locked, then lock
            '''
            Noise
            Model this based on motor specs
            Gaussian ~ N(0,1)
            '''

            self.motors[0].setVelocity(tp.outputs[0][i][1])  
            self.motors[1].setVelocity(tp.outputs[1][i][1])  
            self.motors[2].setVelocity(tp.outputs[2][i][1])
            
            endEffectorLoc = self.endEffectorLocation.getValues()
            traj_li.append(endEffectorLoc)
            jointPosition = [self.ShoulderSensor.getValue(), self.ElbowSensor.getValue(), self.WristSensor.getValue()]
            joint_li.append(jointPosition)
        # Once we reached the target destination, we begin scooping up the tray
        
        if self.save_trajectories == True:
            self.endEffectorTrajectory = pd.DataFrame(np.array(traj_li).round(4))
            self.jointPositionTrajectory = pd.DataFrame(np.array(joint_li).round(4)) 
            saveFileName(self.endEffectorTrajectory, f'{operation_name}_endEffector')
            saveFileName(self.jointPositionTrajectory, f'{operation_name}_jointPosition')

        self.stopMotors()
        
    def collectTray(self, theta1_, theta2_, theta3_, runtime_, 
                    enableSaving=False,
                    noisy_ = False):
        '''
        Simulate the entire process
        Takes the angles theta1_, theta2_, theta3_, runtime, and noisy
        1. theta1_, theta2_, theta3_ : angles of the tray
        2. runtime_ : how long one cycle of the motion will last
        3. noisy_  : adding noisy motors to the motion
        '''
        self.initializeSensors()
        if enableSaving == True:
            self.save_trajectories = True
        
        self.RobotMain.step(self.timeStep)
        initial_positions = np.array(self.determinePosition())
        # While storage is not full and there is a command to pick store
        # in the storage area
        isDone = False
        while(isDone == False):
            # Take a step each time we are in the main loop
            self.RobotMain.step(self.timeStep)

            # Check if the storage unit is full or not
            if self.checkIfStorageIsFull() == False:
                # Moving to the tray
                self.generateVelocities(theta1_, theta2_, theta3_, runtime_,
                                        'move_to_tray')

                # Move vertical lift down
                self.RobotMain.step(self.timeStep)
                self.verticalSliderPosition.enable(self.timeStep)
                self.moveVerticalSlider(VERTICAL_SLIDER_POSITION)
                    
                print('Reached tray')

                # Enable Gripper
                self.gripperTool.leftPosition.enable(self.timeStep)
                self.gripperTool.rightPosition.enable(self.timeStep)
                
                # Grip the tray
                self.gripperTool.contract()
            
                # Move vertical lift up
                self.moveVerticalSlider(0)
                    
                # Move to original location
                self.generateVelocities(np.rad2deg(initial_positions[0]), 
                    np.rad2deg(initial_positions[1]), 
                    np.rad2deg(initial_positions[2]), 
                    runtime_,
                    'move_to_storage')

                # Move down to place the tray by the PLACING_OFFSET
                self.moveVerticalSlider(PLACING_OFFSET)
                self.RobotMain.step(self.timeStep)
                # Release the tray
                self.gripperTool.release()
                
                # Move back to the default resting position
                self.moveVerticalSlider(0)
                
                # Small delay to account for the storage to begin operation
                for i in range(5):
                    self.RobotMain.step(self.timeStep)       
                self.storageCont.storeObject()
                
            # If the storage is full, then we do not do anything
            else:
                pass
            
            isDone = True  
        isDone=False
        return 
    ###################### Saving Motion Analysis  ######################

    def saveEndEffectorTrajectory(self, filename):
        raw_data = self.endEffectorTrajectory
        x = raw_data[:,0]
        y = raw_data[:,2]
        phi = np.sum(self.jointPositionTrajectory, axis=1)
        resultant = np.column_stack((x,y,phi))
        np.savetxt(filename, resultant, delimiter=',')
        
    def saveJointAngleEvolution(self, filename):
        np.savetxt(filename, self.jointPositionTrajectory, delimiter=',')

    
   
# Main Python Routine
# controller = ArmController()
# controller.collectTray(0,0,0,6, True)
# controller.saveEndEffectorTrajectory('webots_trajectory.csv')
# controller.saveJointAngleEvolution('webots_joint.csv')

# detect the current working directory and print it
# path = os.getcwd()
# print(path)
### Storage Testing
# controller.testStorage()
# controller.initializeSensors()