from commands2 import Command, InstantCommand, Subsystem
from wpilib import Timer
from math import pi
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
#        SmartDashboard.putNumber("Net Rot Offset Add",0)
#        SmartDashboard.putNumber("Net Rotation Offset",0)
        self.timer = Timer()
        self.timer.reset()
        self.timer.start()
        self.time1 = 0 
        #  state values:  0 = rotating by stick, 1 = no rot stick and no snap angle, 2 = snap angle               
        self.state =0 

        self.driveTrain = DrivetrainGenerator.getInstance()
        
        self.rotation = 0.0
        self.targetRotation = 0


    def get_rotation_state(self,stick_rot):
        prev_state = self.state
        self.rotation = self.getRotation()-self.driveTrain.get_operator_forward_direction().radians()
        if (abs(stick_rot) > 0):
            self.targetRotation = self.rotation
            self.state=0
            self.time1 = self.timer.get()
        elif self.state != 2 : 
            self.state = 1
            # Capture heading when transitioning from manual rotation to hold mode.
            # This also handles first enable when no rotation has happened yet.
            if prev_state == 0 or self.timer.get() - self.time1 < 0.1:
                self.targetRotation = self.rotation
        return self.state, self.targetRotation


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


    def set_forward_direction(self):
        self.driveTrain.drive_RC(0,0,0)
        self.state=0
        self.driveTrain.set_operator_perspective_forward(
                self.driveTrain.get_state().pose.rotation())
        

    def getRotation(self) -> float:
        return self.driveTrain.get_rotation_rad()
        

    def setTargetRotation(self,angle : float):
        self.targetRotation = angle


#  Returns a command for setting target rotation, for use in auto command sequences and button bindings
    def setTargetRotationCommand(self,angle) -> Command:
        return  InstantCommand(lambda: self.setTargetRotation(angle))   
    
    
# Set target rotation, used for finally_do decorators for commands  that require a supplied boolean.
# The boolean indicates if the command was interrupted or ended naturally
    def setTargetRotationInt (self,b:bool):
        self.state=2
        self.setTargetRotation(self.getRotation()-self.driveTrain.get_operator_forward_direction().radians())

#  Returns a command for setting target rotation, for use in auto command sequences and button bindings
    def setTargetRotationToSelfCommand(self) -> Command:
        return  InstantCommand(lambda: self.setTargetRotationInt(True))   
    
    
