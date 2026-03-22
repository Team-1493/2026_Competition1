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
        self.rotation = self.driveTrain.rotation_rad-self.driveTrain.get_operator_forward_direction().radians()
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


    def rotateToZeroCommand(self):
        return self.runOnce(lambda: self.rotateToZero())
        
    
    def rotateTo90Command(self):
        return self.runOnce(lambda: self.rotateTo90())

    def rotateTo180Command(self):
        return self.runOnce(lambda: self.rotateTo180())

    def rotateTo270Command(self):
        return self.runOnce(lambda: self.rotateTo270())

    def set_forward_direction(self):
        self.driveTrain.drive_RC(0,0,0)
        self.state=0
        self.driveTrain.set_operator_perspective_forward(
                self.driveTrain.pose.rotation())
                

    def set_forward_directionCommand(self):
        return self.runOnce(self.set_forward_direction())

    def setTargetRotation(self,angle : float):
        self.targetRotation = angle


#  Returns a command for setting target rotation, for use in auto command sequences and button bindings
    def setTargetRotationCommand(self,angle) -> Command:
        return  InstantCommand(lambda: self.setTargetRotation(angle))   
    
    
# Set target rotation, used for finally_do decorators for commands  that require a supplied boolean.
# The boolean indicates if the command was interrupted or ended naturally
    def setTargetRotationInt (self,b:bool):
        self.state=2
        self.setTargetRotation(self.driveTrain.rotation_rad-self.driveTrain.get_operator_forward_direction().radians())

#  Returns a command for setting target rotation, for use in auto command sequences and button bindings
    def setTargetRotationToSelfCommand(self) -> Command:
        return  InstantCommand(lambda: self.setTargetRotationInt(True))   
    
    
