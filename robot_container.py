# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands2 import DeferredCommand, InstantCommand
import commands2
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine
from wpimath.geometry import Pose2d
from wpilib import DataLogManager, SmartDashboard, Timer

from Constants1 import ConstantValues
from generated.tuner_constants import TunerConstants
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator 
from telemetry import Telemetry
from subsystems.Drive.heading_controller import HeadingController
from subsystems.Vision.limelight_system import LLsystem
#from subsystems.laser_can import LaserCAN
from subsystems.intake import IntakeSystem
from subsystems.shooter import ShooterSystem
from Commands.drive_teleop_command import DriveTeleopCommand
from Commands.auto_pilot_command import AutoPilotCommand
from Commands.find_wheel_base import FindWheelBase
from Commands.find_ks import FindkS
from Commands.find_slipCurrent import FindSlipCurrent
from Commands.findkP_maxA import FindKP_MaxA
from Commands.arc_drive import arcDrive
from Commands.shoot_command import ShootCommand
from Commands.intake_command import IntakeCommand


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
        self.shoot_command1 = ShootCommand()
        self.shoot_command2 = ShootCommand()        
        self.intake_command = IntakeCommand()            
#        LaserCAN.getInstance()

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
#        self.trigger_arc_drive_near = Trigger(lambda: self.arcdrive.get_is_near())

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

        self._joystick.button(5).onTrue(self.headingController.runOnce(
            lambda:self.headingController.set_forward_direction()))

        self._joystick.button(6).onTrue(self.drivetrain.runOnce(
            lambda:self.drivetrain.reset_pose(Pose2d())))

        self._joystick.button(6).onTrue(self.slow_mode_on)
        
        self._joystick.button(6).onFalse(self.slow_mode_off)


        self._joystick.button(8).whileTrue(self.arcdrive.
            andThen(self.shoot_command1)    
            .finallyDo(self.headingController.setTargetRotationInt) ) 

        self._joystick_op.button(5).whileTrue(self.intake_command)
        self._joystick_op.button(6).whileTrue(self.shoot_command2)        


#        self.trigger_arc_drive_near.onTrue(self.shoot_command)




        
#        self._joystick.button(6).whileTrue(self.shooterSysID_quasi_for)
#        self._joystick.button(7).whileTrue(self.shooterSysID_quasi_rev)
#        self._joystick.button(8).whileTrue(self.shooterSysID_dyn_for)
#        self._joystick.button(9).whileTrue(self.shooterSysID_dyn_rev)
        
#        self._joystick.button(9).onTrue(
#              InstantCommand(lambda:self.update_constants()))




#        self._joystick.button(7).onTrue(
#            InstantCommand(lambda: self.limelightSytem.write_camera0_pose_to_file()))


                
#        self._joystick.button(8).whileTrue(
#            commands2.DeferredCommand(lambda:self.drive_path.drive_trench()).finallyDo
#           (self.headingController.setTargetRotationInt))        



        """    
        self._joystick.button(5).onTrue(
            self.intake.runOnce(lambda:self.intake.arm_up()))
 
        self._joystick.button(6).onTrue(
            self.intake.runOnce(lambda:self.intake.arm_down()))

        self._joystick.button(7).onTrue(
            self.intake.runOnce(lambda:self.intake.intake()))
        
        self._joystick.button(8).onTrue(
            self.intake.runOnce(lambda:self.intake.stop_intake()))
            
        """

#        self._joystick.button(10).onTrue(
#            self.intake.runOnce(lambda:self.intake.zero_position()))

#        self._joystick.button(7).whileTrue(FindkS())
#        self._joystick.button(7).whileTrue(FindSlipCurrent())
#        self._joystick.button(7).whileTrue(FindWheelBase())        
#        self._joystick.button(8).whileTrue(FindKP_MaxA())        

#        self._joystick.button(7).whileTrue(
#            commands2.DeferredCommand(lambda:self.drive_path.drive_path_to_tag(23,-.75,0)).finallyDo
#           (self.headingController.setTargetRotationInt))

#        self._joystick.button(8).whileTrue(
#            AutoPilotCommand(26,-1.5,0,0).finallyDo((self.headingController.setTargetRotationInt)))
 

    def getAutonomousCommand(self):
        return  self.autoChooser.getSelected()


    def setHeadingControlToCurrentrHeading(self):
        self.headingController.setTargetRotationInt(True)  
    
    def write_to_dashboard(self):
#        self.drivetrain.write_to_dashboard()
#        self.intake.write_to_dashboard()
#        self.shooter.write_to_dashboard()
        pass
       
    def apply_teleop_gains(self):
        self.drivetrain.apply_teleop_gains()

    
    def apply_auto_gains(self):
        self.drivetrain.apply_auto_gains()


    
    def update_constants(self):
        # transfer constants from smartdashbaord to constants class        
        self.constants.update_constants()
        # update limelight, autobuilder, and heading controller constants  
        self.limelightSytem.configfureLimelights()
#        self.autoGenerator.configAutoBuilder()
        self.drivetrain.update()
#        self.drive_teleop_command.setConstants()
        self.intake.setup()
        self.shooter.update_constants()
        # DriveGoal_Cam does not need to be explicitly updated, it is generated at each use
    
     
          
    def createPPStuff(self):
        from pathplannerlib.auto import AutoBuilder 
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
            lambda state: self._logger.telemeterize(state)
        )

