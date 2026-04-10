# all distances in meters, all angles in radians

from typing import override
import commands2
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.geometry import Pose2d,Rotation2d,Translation2d
from math import pi
from AutoPilot_py.APTarget import ap_target
from subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain
from AutoPilot_py.AP_Driver import ap_driver
from subsystems.Drive.heading_controller import HeadingController
from Utilities.helper_methods import HelperMethods



class AutoPilotCommand(commands2.Command):

    def __init__(self,m_drivetrain:CommandSwerveDrivetrain):
        self.dt = m_drivetrain
        self.ap_drive = ap_driver.getInstance()

        self.headingController = HeadingController.getInstance()

        self.flag = 1

        self.pose_blue_right_1 =Pose2d(Translation2d(3.4, 2.45),Rotation2d(0)) 
        self.pose_blue_right_2 =Pose2d(Translation2d(6.0, 2.45),Rotation2d(0))

        self.pose_blue_left_1 =Pose2d(Translation2d(3.4, 5.5),Rotation2d(0)) 
        self.pose_blue_left_2 =Pose2d(Translation2d(6.0, 5.5),Rotation2d(0))

        self.pose_red_right_1 =Pose2d(Translation2d(16.54-3.4, 5.5),Rotation2d(pi)) 
        self.pose_red_right_2 =Pose2d(Translation2d(16.54 - 6.0, 5.5),Rotation2d(pi))

        self.pose_red_left_1 =Pose2d(Translation2d(16.54 - 3.4, 2.45),Rotation2d(pi)) 
        self.pose_red_left_2 =Pose2d(Translation2d(16.54 - 6.0, 2.45),Rotation2d(pi))

        self.targetPose1 = Pose2d()
        self.targetPose1 = Pose2d()
        self.entryangle1 = 0
        self.entryangle2 = 0
        self.ang1=0
        self.ang2=0
        self.vfinal1 = 0
        self.vfinal2 = 0        
        self.v1 = 2
        self.addRequirements(self.dt)    
    

    @override
    def initialize(self):
        self.targetPose1 = None

        currentX = self.dt.pose.X()
        currentY = self.dt.pose.Y()
        if DriverStation.getAlliance()==DriverStation.Alliance.kBlue:
            if currentY<4.03:
                if currentX<=3.4:
                    self.targetPose1 = self.pose_blue_right_1
                    self.entryangle1=self.ang1
                    self.vfinal1=self.v1
                    self.targetPose2 = self.pose_blue_right_2
                    self.entryangle2=self.ang2
                    self.vfinal2=0

                elif currentX<=4.1:
                    self.targetPose1 = self.pose_blue_right_2
                    self.entryangle1=self.ang1
                    self.vfinal1=0
                    self.targetPose2=None


                elif currentX<=5.2:
                    self.targetPose1 = self.pose_blue_right_1
                    self.entryangle1=self.ang1+pi
                    self.vfinal1=0
                    self.targetPose2=None


                else:
                    self.targetPose1 = self.pose_blue_right_2
                    self.entryangle1=self.ang1+pi
                    self.vfinal1=self.v1
                    self.targetPose2 = self.pose_blue_right_1
                    self.entryangle2=self.ang2+pi
                    self.vfinal2=0


            else:
                if currentX<=3.4:
                    self.targetPose1 = self.pose_blue_left_1
                    self.entryangle1=self.ang1
                    self.vfinal1=self.v1
                    self.targetPose2 = self.pose_blue_left_2
                    self.entryangle2=self.ang2
                    self.vfinal2=0

                elif currentX<=4.1:
                    self.targetPose1 = self.pose_blue_left_2
                    self.entryangle1=self.ang1
                    self.vfinal1=0
                    self.targetPose2 = None


                elif currentX<=5.2:
                    self.targetPose1 = self.pose_blue_left_1
                    self.entryangle1=self.ang1
                    self.vfinal1=0
                    self.targetPose2 = None

                else:
                    self.targetPose1 = self.pose_blue_left_2
                    self.entryangle1=self.ang1+pi
                    self.vfinal1=0
                    self.targetPose2 = self.pose_blue_left_1
                    self.entryangle2=self.ang1+pi
                    self.vfinal2=0



        else:
            if currentY<4.03:
                if currentX>=16.54 - 3.4:
                    self.targetPose1 = self.pose_red_left_1
                    self.entryangle1=self.ang1+pi
                    self.vfinal1=self.v1
                    self.targetPose2 = self.pose_red_left_2
                    self.entryangle2=self.ang1+pi
                    self.vfinal2=0
                    

                elif currentX>=16.54 - 4.1:
                    self.targetPose1 = self.pose_red_left_2
                    self.entryangle1=self.ang1+pi
                    self.vfinal1=0
                    self.targetPose2 = 2

                elif currentX>=16.54 - 5.2:
                    self.targetPose1 = self.pose_red_left_1
                    self.entryangle1=self.ang1
                    self.vfinal1=0
                    self.targetPose2 = 0

                else:
                    self.targetPose1 = self.pose_red_left_2
                    self.entryangle1=self.ang1
                    self.vfinal1=self.v1
                    self.targetPose2 = self.pose_red_left_1
                    self.entryangle2=self.ang1
                    self.vfinal2=0

            else:
                if currentX>=16.54 -3.4:
                    self.targetPose1 = self.pose_red_right_1
                    self.entryangle1=self.ang1+pi
                    self.vfinal1=self.v1
                    self.targetPose2 = self.pose_red_right_2
                    self.entryangle2=self.ang1+pi
                    self.vfinal2=0

                elif currentX>=16.54 -4.1:
                    self.targetPose1 = self.pose_red_right_2
                    self.entryangle1=self.ang1+pi
                    self.vfinal1=0
                    self.targetPose2=None


                elif currentX>=16.54 -5.2:
                    self.targetPose1 = self.pose_red_right_1
                    self.entryangle1=self.ang1
                    self.vfinal1=0
                    self.targetPose2 = None


                else:
                    self.targetPose1 = self.pose_red_right_1
                    self.entryangle1=self.ang1
                    self.vfinal1=self.v1
                    self.targetPose2 = self.pose_red_right_2
                    self.entryangle2=self.ang1+pi
                    self.vfinal2=0

        self.pose = self.targetPose1
        self.entryAngle = Rotation2d(self.entryangle1)
        self.velFinal = self.vfinal1
        self.m_target = ap_target(self.pose).with_entry_angle(self.entryAngle).with_velocity(self.velFinal)
        
        
    @override
    def execute(self):

        out,disp = self.ap_drive.kAutopilot.calculate(
            self.dt.pose, self.dt.speeds, self.m_target)
        
                    
        self.dt.drive_autopilot(out.vx,out.vy,out.targetAngle.radians())
  

    @override
    def isFinished(self):
        done = self.ap_drive.kAutopilot.atTarget(
            self.dt.pose, 
            self.m_target)

        if (done and self.flag==2):
            return True
        elif (done and self.targetPose2 == None):
            return True
        elif (done and self.flag == 1):
            self.flag ==2
            self.pose = self.targetPose2
            self.entryAngle = Rotation2d(self.entryangle2)
            self.velFinal = self.vfinal2
            self.m_target = ap_target(self.pose).with_entry_angle(self.entryAngle).with_velocity(self.velFinal)
            return False
        else:
            return False

  

    @override
    def end(self,interrupted:bool):
        self.dt.drive_RC(0,0,0)
#        print("done with autopilot")
