from commands2 import Command, InstantCommand, Subsystem
from wpilib import SmartDashboard, Timer
from wpimath.controller  import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from math import pi,hypot
from Constants1 import ConstantValues
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator 

#To Do - check if rotation not being controlled until first rotation

class HeadingController(Subsystem):
    instance=None
    
    @staticmethod
    def getInstance():
        if HeadingController.instance == None:
            HeadingController.instance = HeadingController()
            print("**********************  HEADING CONTROLLER  **********************") 
        return HeadingController.instance
    
    def __init__(self):
        self.timer = Timer()
        self.timer.reset()
        self.timer.start()
        self.time1 = 0        
        self.state =0 #  0 = rotating by stick, 1 = no rot stick and no snap angle, 2 = snap angle

        self.driveTrain = DrivetrainGenerator.getInstance()
        
        self.rotation = 0.0
        self.targetRotation = 0.0


    def get_rotation_state(self,stick_rot):
        dir=self.driveTrain.get_operator_forward_direction().radians()
        self.rotation = self.getRotation()

        if abs(stick_rot) > 0:
            self.targetRotation = self.rotation+dir
            self.state=0
            self.time1 = self.timer.get()
        elif self.state != 2 : 
            self.state = 1
            if self.timer.get() - self.time1<0.1:
                self.targetRotation = self.rotation+dir
        return self.state,self.targetRotation


    def setTargetRotation(self,angle : float):
        self.targetRotation = angle


    def rotateToZero(self):
        self.setTargetRotation(0)
        self.state=2

    def rotateTo90(self):
        self.setTargetRotation(pi/2)
        self.state=2

    def rotateTo180(self):
        self.setTargetRotation(pi)
        self.state=2

    def rotateTo270(self):
        self.setTargetRotation(3*pi/2)
        self.state=2        

    def zeroStateAngle(self):
        self.state=0
        self.setTargetRotation(0)
        


    def getRotation(self) -> float:
        return self.driveTrain.get_rotation_rad()
    
     #self.driveTrain.get_state().pose.rotation().radians()
    

    def setTargetRotationCommand(self,angle) -> Command:
        return  InstantCommand(lambda: self.setTargetRotation(angle))   
    
    def setTargetRotationInt (self,b:bool):
        self.state=2
        self.setTargetRotation(self.getRotation()+self.driveTrain.get_operator_forward_direction().radians())

    
    
