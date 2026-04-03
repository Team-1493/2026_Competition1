#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math
import commands2
import typing

from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from robot_container import RobotContainer
from pathplannerlib.auto import AutoBuilder,PathPlannerAuto

from pathplannerlib.path import PathPlannerPath, PathPlannerTrajectory


autonomousCommand: typing.Optional[commands2.Command] = None


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """
    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        self.container = RobotContainer()  
        self.IMU_mode = 0
        self.print_counter=0

    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""

        # Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        # commands, running already-scheduled commands, removing finished or interrupted commands,
        # and running subsystem periodic() methods.  This must be called from the robot's periodic
        # block in order for anything in the Command-based framework to work.
        commands2.CommandScheduler.getInstance().run()
        self.print_counter=self.print_counter+1
        if self.print_counter>=0:
            self.print_counter=0
            self.container.write_to_dashboard()
            
    def disabledInit(self) -> None:
        pass


    def disabledPeriodic(self) -> None:
        self.container.limelightSytem.zeroAndseedIMU()
        pass


    def autonomousInit(self) -> None:
        self.container.apply_auto_gains()        
        self.container.limelightSytem.set_IMU_Mode(self.IMU_mode)
        self.autonomousCommand = self.container.getAutonomousCommand()
        
        if self.autonomousCommand:
            print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA  COMMAND SCHEDULED: ", self.autonomousCommand.getName())
            allPaths = PathPlannerAuto.getPathGroupFromAutoFile(self.autonomousCommand.getName())
            firstPath = allPaths[0]  # the first path in your PathPlanner Auto
            autoStartPose = firstPath.getStartingHolonomicPose()
            self.initializePose(autoStartPose)          
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        pass


    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
        self.initializePose(None)            
        self.container.setHeadingControlToCurrentrHeading()
        self.container.apply_teleop_gains()
        # Reseed Limelight IMU from the latest drivetrain heading at the
        # autonomous -> teleop transition.
        self.container.limelightSytem.zeroAndseedIMU()        
        
        #  0 for external IMU,  2 for internal IMU, 4 for internal + externa assist         
        self.container.limelightSytem.set_IMU_Mode(self.IMU_mode)   


    def teleopPeriodic(self) -> None:
        pass


    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.setHeadingControlToCurrentrHeading()
    


    @typing.override
    def _simulationInit(self):
        from subsystems.Vision.photon_vision_sim import PVisionSim
        vsim = PVisionSim()
    


    def initializePose(self, autoStartPose: Pose2d | None = None):
        """
        Initialize the robot's pose for the match.

        Handles:
        1. Autonomous start (autoStartPose provided)
        2. Teleop-only start (no auto run)
        3. Random boot angles, driver manually rotates robot
        4. Red/Blue alliance flip for downfield orientation
        """

    # Determine alliance offset
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            allianceOffset = Rotation2d.fromDegrees(180)
        else:
            allianceOffset = Rotation2d.fromDegrees(0)

        if autoStartPose is not None:
            # Case 1: Auto is running
            #  PathPlanner already provides autoStartPose in BLUE field coords
            # Apply alliance flip by adding offset
            finalPose = Pose2d(
                autoStartPose.translation(),
                autoStartPose.rotation() + allianceOffset
            )
            self.auto_has_run = True

        else:
            # Case 2: Teleop-only start
            # Capture current driver-set heading from gyro
            currentRotation = Rotation2d(self.container.drivetrain.pigeon2.get_yaw().value_as_double)            finalRotation = currentRotation + allianceOffset

            # Use placeholder translation (0,0), vision will correct X/Y
            finalPose = Pose2d(Translation2d(0, 0), finalRotation)
            self.auto_has_run = False

        # Reset the pose estimator
        self.container.drivetrain.reset_pose(finalPose)

        # Debug print
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA  ",DriverStation.getAlliance().name,"  ",
              finalPose.rotation().degrees())
            