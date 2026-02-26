import math
from typing import override
import commands2
from wpilib import SmartDashboard, Timer
import wpimath
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from phoenix6 import swerve
from  subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain

class FindKP_MaxA(commands2.Command):
    def __init__(self):
        self.drivetrain = DrivetrainGenerator.getInstance()
        self.addRequirements(self.drivetrain)
        self.timer = Timer()
        self.accel = 1.5  #m/s/s
        
    @override
    def initialize(self):
        self.timer.reset()
        self.timer.start()
        self.timePrevious = 0
        self.vel_measured = 0
        self.vel_requested = 0
        SmartDashboard.putNumber("Vel - measured",self.vel_measured)
        SmartDashboard.putNumber("Vel - requested",self.vel_requested)
      

    def execute(self) -> None:
        timeCurrent = self.timer.get()
        deltaTime = timeCurrent - self.timePrevious
        self.timePrevious = timeCurrent
        self.vel_requested += deltaTime*self.accel
        self.drivetrain.drive_FC_facing(self.vel_requested,0,0)
        self.vel_measured=self.drivetrain.get_speeds_norm()
        SmartDashboard.putNumber("Vel - measured",self.vel_measured)
        SmartDashboard.putNumber("Vel - requested",self.vel_requested)



    @override
    def end(self, interrupted: bool) :
        self.drivetrain.drive_FC_facing(0,0,0)

    @override
    def isFinished(self):
        return False
