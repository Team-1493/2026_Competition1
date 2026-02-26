import typing
import commands2
from math import copysign
from wpimath.geometry import Rotation2d
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from generated.tuner_constants import TunerConstants
from subsystems.Drive.heading_controller import HeadingController
from phoenix6 import swerve
from wpimath.units import rotationsToRadians
from wpilib import SmartDashboard
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
      
        self.request_teleop_FC = (
            swerve.requests.FieldCentric()
            .with_deadband(ConstantValues.DriveConstants.SPEED_AT_12_VOLTS *
                    ConstantValues.DriveConstants.TELEOP_DEADBAND)  #squared input, so db starts at 0.05
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY)
        )

        self.request_teleop_FC_facing = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(ConstantValues.DriveConstants.SPEED_AT_12_VOLTS *
                    ConstantValues.DriveConstants.TELEOP_DEADBAND)  #squared input, so db starts at 0.05
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_heading_pid(
                ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KP,
                0,
                ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KD)    
        )

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

        self.requested_velocity=forw*self._max_speed*self.scale_factorXY
        SmartDashboard.putNumber("requested velocity", self.requested_velocity)
        dir = self.drivetrain.get_operator_forward_direction().radians()
        state, target_angle = self.headingController.get_rotation_state(rot*self._max_angular_rate)    
#        state =0.
#        target_angle=0   
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
        

    def setConstants(self):
        self._max_speed = (ConstantValues.DriveConstants.SPEED_AT_12_VOLTS)  
        self._max_angular_rate = rotationsToRadians(ConstantValues.DriveConstants.TELEOP_MAX_ANGULAR_RATE)  
        self.scale_factorXY = ConstantValues.DriveConstants.TELEOP_SCALE_FACTOR_XY
        self.scale_factorRot = ConstantValues.DriveConstants.TELEOP_SCALE_FACTOR_ROT
        
