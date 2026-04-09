# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands2 import DeferredCommand, InstantCommand
import commands2
from commands2.button import CommandXboxController
from wpimath.geometry import Pose2d,Rotation2d,Translation2d
from wpilib import DataLogManager, SmartDashboard, Timer
from pathplannerlib.auto import AutoBuilder 

from Constants1 import ConstantValues
from generated.tuner_constants import TunerConstants
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator 
from telemetry import Telemetry
from subsystems.Drive.heading_controller import HeadingController
from subsystems.Vision.limelight_system import LLsystem
from subsystems.intake import IntakeSystem
from subsystems.shooter import ShooterSystem
from Commands.agitate_intake import AgitateIntake
from Commands.drive_teleop_command import DriveTeleopCommand
from Commands.auto_pilot_command import AutoPilotCommand
from Commands.drive_over_trench import DriveOveTrench
from Commands.arc_drive import arcDrive
from Commands.shoot_command import ShootCommand
from Commands.intake_command import IntakeCommand
from subsystems.led import led_system

class RobotContainer:

    def __init__(self) -> None:
        ""
    
#        self.timer = Timer()
#        self.timer.reset()
#        self.timer.start()

        self.constants = ConstantValues.getInstance()
 #       while self.timer.get()<3:""
#            print("Waiting for Warmup",round(self.timer.get(),0))
        self.drivetrain = DrivetrainGenerator.getInstance()
 #       while self.timer.get()<5:""
#            print("Creating CAN Devices",round(self.timer.get(),0))

        self.headingController = HeadingController.getInstance()        
        self.intake = IntakeSystem.getInstance()
        self.shooter = ShooterSystem.getInstance()   
#        self.LED =  led_system()
        self.shoot_command = ShootCommand()
        self.intake_command = IntakeCommand()  
        self.agitate_command = AgitateIntake()          
        self.over_trench = DriveOveTrench()

        self.limelightSytem = LLsystem.getInstance()
        self._joystick = CommandXboxController(0)
        self._joystick_op = CommandXboxController(1)

        # speed_at_12_volts desired top speed
        self._max_speed = (TunerConstants.speed_at_12_volts) 
        self.drive_teleop_command = DriveTeleopCommand(self.drivetrain,
                lambda: -self._joystick.getRawAxis(1),
                lambda: -self._joystick.getRawAxis(0),
                lambda: -self._joystick.getRawAxis(4))
        
        self.arcdrive = arcDrive(self.drivetrain)

        self.rotateToZero = self.headingController.rotateToZeroCommand()
        self.rotateTo90 = self.headingController.rotateTo90Command()
        self.rotateTo180 = self.headingController.rotateTo180Command()
        self.rotateTo270 = self.headingController.rotateTo270Command()   
        
        self.slow_mode_on = InstantCommand(lambda:self.drive_teleop_command.slow_mode_on())
        self.slow_mode_off = InstantCommand(lambda:self.drive_teleop_command.slow_mode_off())        
#        self.setForwardDirection = self.headingController.set_forward_directionCommand()                     
        self.createPPStuff()
        self.set_up_telemetry()
        self.configureButtonBindings()


    def configureButtonBindings(self) -> None:

        self.drivetrain.setDefaultCommand(self.drive_teleop_command)
       
        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
#        idle = swerve.requests.Idle()
##        Triggerc(DriverStation.isDisabled).whileTrue(
#            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
#        )

        self._joystick.button(4).onTrue(self.rotateToZero)
        
        self._joystick.button(3).onTrue(self.rotateTo90)

        self._joystick.button(1).onTrue(self.rotateTo180)

        self._joystick.button(2).onTrue(self.rotateTo270)

#        self._joystick.button(5).onTrue(self.headingController.runOnce(
#            lambda:self.headingController.set_forward_direction()))


        self._joystick.button(6).onTrue(self.slow_mode_on)
        
        self._joystick.button(6).onFalse(self.slow_mode_off)

#        self._joystick.button(7).whileTrue(DeferredCommand(lambda:self.drive_path.drive_trench()))
#        self._joystick.button(7).onTrue(self.drivetrain.runOnce(lambda:self.drivetrain.reset_pose(Pose2d(Translation2d(0.3,0.66),Rotation2d(0)))))
        self._joystick.button(8).whileTrue(self.arcdrive    
            .finallyDo(self.headingController.setTargetRotationInt) ) 

        self._joystick_op.button(5).whileTrue(self.intake_command)
        self._joystick_op.button(6).whileTrue(self.shoot_command)
        self._joystick_op.button(7).whileTrue(self.agitate_command)                
        
        self._joystick.button(9).onTrue(
              InstantCommand(lambda:self.update_constants()))

        self._joystick.button(7).whileTrue(commands2.DeferredCommand(
            lambda:self.over_trench.drivetrench()))
 

    def getAutonomousCommand(self):
#        return PathPlannerAuto(self.autoChooser.getSelected())            
        return  self.autoChooser.getSelected()


    def setHeadingControlToCurrentrHeading(self):
        self.headingController.setTargetRotationInt(True)  
    
    def write_to_dashboard(self):
        self.drivetrain.write_to_dashboard()
        self.intake.write_to_dashboard()
#        self.shooter.write_to_dashboard()
#        pass
       
    def apply_teleop_gains(self):
        self.drivetrain.apply_teleop_gains()

    
    def apply_auto_gains(self):
        self.drivetrain.apply_auto_gains()


    
    def update_constants(self):
        # transfer constants from smartdashbaord to constants class        
        self.constants.update_constants()
        # update limelight, autobuilder, and heading controller constants  
        self.limelightSytem.configfureLimelights()
        self.autoGenerator.configAutoBuilder()
        self.autoChooser = None
        self.autoChooser = AutoBuilder.buildAutoChooser("DoNothing")   
        SmartDashboard.putData("Auto Chooser", self.autoChooser)             
        self.drivetrain.update()
        self.drive_teleop_command.setConstants()
        self.intake.setup()
        self.shooter.update_constants()
     
          
    def createPPStuff(self):
        from Auto.auto_generator import AutoGenerator 
        from Commands.drive_path_generator import DrivePathGenerator 
        self.autoGenerator = AutoGenerator()
        self.autoChooser = AutoBuilder.buildAutoChooser("DoNothing")
        SmartDashboard.putData("Auto Chooser", self.autoChooser)
        self.drive_path = DrivePathGenerator(
                 lambda: self.drivetrain.pose_supplier())
        
    def set_up_telemetry(self):
        self._logger = Telemetry(TunerConstants.speed_at_12_volts)
        DataLogManager.start()
        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state))
