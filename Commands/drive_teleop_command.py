import typing
import commands2
from math import copysign
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from generated.tuner_constants import TunerConstants
from subsystems.Drive.heading_controller import HeadingController
from phoenix6 import swerve
from wpimath.units import rotationsToRadians
from  subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain
from Constants1 import ConstantValues



class DriveTeleopCommand(commands2.Command):    
    def __init__(
        self,
        _drivetrain: CommandSwerveDrivetrain,
        forward: typing.Callable[[], float],
        side: typing.Callable[[], float],
        rotate: typing.Callable[[], float],
    ) -> None:
        super().__init__()
        self.requested_velocity = 0
        self.forward = forward
        self.side = side
        self.rotate = rotate
        self.drivetrain = _drivetrain
        self.headingController = HeadingController.getInstance()
        self.slow_mode = 1
        self.setConstants()
        self.addRequirements(self.drivetrain)




    def execute(self) -> None:
        forw=self.forward()
        sde=self.side()
        rot = self.rotate()

        if rot<0.05 and rot>-0.05: rot=0
        if forw<0.05 and forw>-0.05: forw=0
        if sde<0.05 and sde>-0.05: sde=0                
        # square input
        forw = copysign(forw**2,forw)
        sde = copysign(sde**2,sde)
        rot = copysign(rot**2,rot)

        state, target_angle = self.headingController.get_rotation_state(rot*self._max_angular_rate)  
      
        self.scale_factorXY=self.slow_mode*SmartDashboard.getNumber("Drive Teleop Scale XY", self.scale_factorXY)
        self.scale_factorRot=self.slow_mode*SmartDashboard.getNumber("Drive Teleop Scale Rot", self.scale_factorRot)

        if state==0:
            self.drivetrain.drive_FC(
                forw*self._max_speed*self.scale_factorXY,
                sde*self._max_speed*self.scale_factorXY,
                rot*self._max_angular_rate*self.scale_factorRot)
            
        else:
            self.drivetrain.drive_FC_facing(
                forw*self._max_speed*self.scale_factorXY,
                sde*self._max_speed*self.scale_factorXY,
                target_angle)    
        

    def slow_mode_on(self):
        self.slow_mode=.5

    def slow_mode_off(self):
        self.slow_mode=1

        

    def setConstants(self):
        self._max_speed = (ConstantValues.DriveConstants.SPEED_AT_12_VOLTS)  
        self._max_angular_rate = rotationsToRadians(ConstantValues.DriveConstants.TELEOP_MAX_ANGULAR_RATE)  
        self.scale_factorXY = ConstantValues.DriveConstants.TELEOP_SCALE_FACTOR_XY
        self.scale_factorRot = ConstantValues.DriveConstants.TELEOP_SCALE_FACTOR_ROT

        
