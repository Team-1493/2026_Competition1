#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import commands2
import typing

from robot_container import RobotContainer
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

    def disabledInit(self) -> None:
        pass


    def disabledPeriodic(self) -> None:
        self.container.limelightSytem.zeroAndseedIMU()
        pass


    def autonomousInit(self) -> None:
        self.container.limelightSytem.set_IMU_Mode(self.IMU_mode)
        self.autonomousCommand = self.container.getAutonomousCommand()
        if self.autonomousCommand:
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
        self.container.setHeadingControlToCurrentrHeading()
        
        #  0 for external IMU,  2 for internal IMU, 4 for internal + externa assist         
        self.container.limelightSytem.set_IMU_Mode(self.IMU_mode)   


    def teleopPeriodic(self) -> None:
        pass


    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
        self.container.setHeadingControlToCurrentrHeading()
    


#    @typing.override
#    def _simulationInit(self):
#        from subsystems.Vision.photon_vision_sim import PVisionSim
#        vsim = PVisionSim()
    