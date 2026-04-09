# all distances in meters, all angles in radians

from typing import override
import commands2
from AutoPilot_py.APTarget import ap_target
from AutoPilot_py.AP_Driver import ap_driver
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from Utilities.helper_methods import HelperMethods
from wpimath.geometry import Pose2d, Rotation2d

class AutoPilotCommand(commands2.Command):

    def __init__(self,pose:Pose2d,entryAngle,velFInal = 0,
                 positions:list = None,actions:list[commands2.Command]=None):
        print("******* AUTOPILOT COMMAND   ********")
        self.pose = pose
        self.entryAngle = Rotation2d(entryAngle)
        self.velFinal = velFInal
        self.m_drivetrain = DrivetrainGenerator.getInstance()
        self.addRequirements(self.m_drivetrain)    

    @override
    def initialize(self):
#        print("****STARTING AUTOPILOT")
        self.ap_drive = ap_driver.getInstance()
        self.m_target = ap_target(self.pose).with_entry_angle(self.entryAngle).with_velocity(self.velFinal)
  

    @override
    def execute(self):

        out,disp = self.ap_drive.kAutopilot.calculate(
            self.m_drivetrain.pose, self.m_drivetrain.speeds, self.m_target)
        
                    
        self.m_drivetrain.drive_autopilot(out.vx,out.vy,out.targetAngle.radians())
  

    @override
    def isFinished(self):
        return self.ap_drive.kAutopilot.atTarget(
            self.m_drivetrain.pose, 
            self.m_target)
  

    @override
    def end(self,interrupted:bool):
        self.m_drivetrain.drive_RC(0,0,0)
#        print("done with autopilot")
