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
        self.actions = actions
        self.positions = positions
        if self.actions is not None:
            self.actions_length = len(self.actions)
            self.hasActions = True
        else:
            self.actions_length = 0 
            self.hasActions = False

        self.addRequirements(self.m_drivetrain)    

    @override
    def initialize(self):
#        print("****STARTING AUTOPILOT")
        self.ap_drive = ap_driver.getInstance()
        self.m_target = ap_target(self.pose).with_entry_angle(self.entryAngle).with_velocity(self.velFinal)
        self.index_actions = 0
 #       print("************",self.m_target.get_reference().X(),"  ",self.m_target.get_reference().Y())
  

    @override
    def execute(self):

        out,disp = self.ap_drive.kAutopilot.calculate(
            self.m_drivetrain.pose, self.m_drivetrain.speeds, self.m_target)
        
        if self.hasActions:
            if self.index_actions<self.actions_length:
                if disp>self.positions[self.index_actions]:
                    self.actions[self.index_actions].schedule()
                    self.index_actions = self.index_actions+1
                    
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
