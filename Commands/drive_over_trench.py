from wpilib import DriverStation
from math import pi
from wpimath.geometry import Pose2d,Rotation2d,Translation2d
from subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain
from Commands.auto_pilot_command import AutoPilotCommand
from subsystems.Drive.heading_controller import HeadingController
from commands2 import SequentialCommandGroup

class DriveOveTrench():
    def __init__(self,m_dt:CommandSwerveDrivetrain) -> None:
        super().__init__()
        self.dt = m_dt
        print("BBBBBBBBBBBBBBBBBB ",  self.dt)
        self.headingController = HeadingController.getInstance()

        self.pose_blue_right_1 =Pose2d(Translation2d(3.4, 2.45),Rotation2d(0)) 
        self.pose_blue_right_2 =Pose2d(Translation2d(6.0, 2.45),Rotation2d(0))

        self.pose_blue_left_1 =Pose2d(Translation2d(3.4, 5.5),Rotation2d(0)) 
        self.pose_blue_left_2 =Pose2d(Translation2d(6.0, 5.5),Rotation2d(0))

        self.pose_red_right_1 =Pose2d(Translation2d(16.54-3.4, 5.5),Rotation2d(pi)) 
        self.pose_red_right_2 =Pose2d(Translation2d(16.54 - 6.0, 5.5),Rotation2d(pi))

        self.pose_red_left_1 =Pose2d(Translation2d(16.54 - 3.4, 2.45),Rotation2d(pi)) 
        self.pose_red_left_2 =Pose2d(Translation2d(16.54 - 6.0, 2.45),Rotation2d(pi))


    def drivetrench(self):
        currentX = self.dt.pose.X()
        currentY = self.dt.pose.Y()
        if DriverStation.getAlliance()==DriverStation.Alliance.kBlue:
            if currentY<4.03:
                if currentX<=3.4:
                    return SequentialCommandGroup(
                        AutoPilotCommand(self.dt,self.pose_blue_right_1,0,2 ),
                        AutoPilotCommand(self.dt,self.pose_blue_right_2,0,0)).finallyDo(
                        (self.headingController.setTargetRotationInt))
                elif currentX<=4.1:
                    return AutoPilotCommand(self.dt,self.pose_blue_right_2,0,0).finallyDo(self.headingController.setTargetRotationInt)
                elif currentX<=5.2:
                    return AutoPilotCommand(self.dt,self.pose_blue_right_1,pi,0 ).finallyDo(
                    (self.headingController.setTargetRotationInt))
                else:
                    return SequentialCommandGroup(
                        AutoPilotCommand(self.dt,self.pose_blue_right_2,pi,2 ),
                        AutoPilotCommand(self.dt,self.pose_blue_right_1,pi,0)).finallyDo(
                        (self.headingController.setTargetRotationInt))
            else:
                if currentX<=3.4:
                    return AutoPilotCommand(
                    self.dt,self.pose_blue_left_1,0,2 ).andThen(
                    AutoPilotCommand(self.dt,self.pose_blue_left_2,0,0)).finallyDo(
                    (self.headingController.setTargetRotationInt))
                elif currentX<=4.1:
                    return AutoPilotCommand(self.dt,self.pose_blue_left_2,0,0).finallyDo(
                    self.headingController.setTargetRotationInt)
                elif currentX<=5.2:
                    return AutoPilotCommand(
                    self.dt,self.pose_blue_left_1,pi,0 ).finallyDo(
                    (self.headingController.setTargetRotationInt))
                else:
                    return AutoPilotCommand(
                    self.dt,self.pose_blue_left_2,pi,2 ).andThen(
                    AutoPilotCommand(self.dt,self.pose_blue_left_1,pi,0)).finallyDo(
                    (self.headingController.setTargetRotationInt))
        else:
            if currentY<4.03:
                if currentX>=16.54 - 3.4:
                    return AutoPilotCommand(
                    self.dt,self.pose_red_left_1,pi,2 ).andThen(
                    AutoPilotCommand(self.dt,self.pose_red_left_2,pi,0)).finallyDo(
                    (self.headingController.setTargetRotationInt))
                elif currentX>=16.54 - 4.1:
                    return AutoPilotCommand(self.dt,self.pose_red_left_2,pi,0).finallyDo(
                    (self.headingController.setTargetRotationInt))
                elif currentX>=16.54 - 5.2:
                    return AutoPilotCommand(
                    self.dt,self.pose_red_left_1,0,0 ).finallyDo(
                    (self.headingController.setTargetRotationInt))
                else:
                    return AutoPilotCommand(
                    self.dt,self.pose_red_left_2,0,2 ).andThen(
                    AutoPilotCommand(self.dt,self.pose_red_left_1,0,0)).finallyDo(
                    (self.headingController.setTargetRotationInt))
            else:
                if currentX>=16.54 -3.4:
                    return AutoPilotCommand(
                    self.dt,self.pose_red_right_1,pi,2 ).andThen(
                    AutoPilotCommand(self.dt,self.pose_red_right_2,pi,0)).finallyDo(
                    (self.headingController.setTargetRotationInt))
                elif currentX>=16.54 -4.1:
                    return AutoPilotCommand(self.dt,self.pose_red_right_2,pi,0).finallyDo(
                    (self.headingController.setTargetRotationInt))
                elif currentX>=16.54 -5.2:
                    return AutoPilotCommand(
                    self.dt,self.pose_red_right_1,0,0 ).finallyDo(
                    (self.headingController.setTargetRotationInt))
                else:
                    return AutoPilotCommand(
                    self.dt,self.pose_red_right_1,0,2 ).andThen(
                    AutoPilotCommand(self.dt,self.pose_red_right_1,pi,0)).finallyDo(
                    (self.headingController.setTargetRotationInt))
