import math
from typing import override
import typing
import commands2
from wpilib import DriverStation, SmartDashboard
import wpimath
from Commands.stop_drive import StopDrive
from subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from phoenix6 import swerve
from wpimath.geometry import Pose2d,Rotation2d
from Commands.drive_path_generator import DrivePathGenerator
from pathplannerlib.config  import PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController 
from pathplannerlib.trajectory import PathPlannerTrajectoryState;
from wpimath.kinematics import ChassisSpeeds



class arcDrive(commands2.Command):
    def __init__(self,
                _driveTrain:CommandSwerveDrivetrain
                # _targetPose: typing.Callable[[], Pose2d],
                ) -> None:
        super().__init__()
        self.driveTrain = _driveTrain
        self.radius = 3
        self.m=2.0#max speed

        self.isFinishedFlag = False
        self.addRequirements(self.driveTrain)
        self.kTranslationPID = PIDConstants(4.0,0,0)
        self.kRotationPID = PIDConstants(4.0,0,0)
        self.controller = PPHolonomicDriveController(
            self.kTranslationPID, self.kRotationPID)

        
   
        

    @override
    def initialize(self):
        currentPose = self.driveTrain.get_state().pose
        xr = currentPose.X()
        yr = currentPose.Y()

        self.validLocation=True

        # Set correct hub location for alliance
        if DriverStation.getAlliance()==DriverStation.Alliance.kBlue:
            self.xh = 4.644
            self.yh= 4.030
            if xr>4:
                self.validLocation=False
            if xr>3.3:
                xr = 3

        else:
            self.xh = 11.92
            self.yh= 4.041
            if xr<12.7:
                self.validLocation=False
            if xr<13.4:
                xr=13.7

        xr1=(self.xh - xr)
        yr1 = self.yh - yr

        xt=-yr1
        yt=xr1

        xt_goal=self.radius*xr1/math.sqrt(xr1*xr1 + yr1*yr1)
        yt_goal=self.radius*yr1/math.sqrt(xr1*xr1 + yr1*yr1)

        self.y_goal=self.yh - yt_goal
        self.x_goal=self.xh -xt_goal

        self.angle_goal=math.atan2(yt,xt)-math.pi/2  

        self.goalPose = Pose2d(self.x_goal,self.y_goal,Rotation2d(self.angle_goal)) 
        self.goalState = PathPlannerTrajectoryState()
        self.goalState.pose = self.goalPose
        print(xr,"  ",yr,"  ",xr1,"  ",yr1,"  ",self.x_goal,"  ",self.y_goal,"  ",self.angle_goal )

    def execute(self):
        speeds =self.controller.calculateRobotRelativeSpeeds(
            self.driveTrain.get_pose(), self.goalState)

        speeds = ChassisSpeeds(math.sqrt(self.m)*(speeds.vx/(math.sqrt(speeds.vx**2 + speeds.vy**2))),math.sqrt(self.m)*(speeds.vy/(math.sqrt(speeds.vx**2 + speeds.vy**2))),
                speeds.omega)

        if self.validLocation:
            self.driveTrain.drive_RC(speeds.vx,speeds.vy,speeds.omega)


    @override
    def isFinished(self):
        if self.validLocation is False:
            return True

        current = self.driveTrain.get_pose()
        xyErr =math.hypot(current.X() - self.x_goal,current.Y() - self.y_goal) 
        okXY = xyErr <= .04
       
        thetaErr = abs(current.rotation().radians() - self.angle_goal)
        if thetaErr>math.pi: thetaErr = thetaErr - 2*math.pi
        if thetaErr<-math.pi: thetaErr = thetaErr - 2*math.pi 
        okTheta = abs(thetaErr) <= .04
        return okXY and okTheta
  

    @override
    def end(self,interrupted:bool):
        self.driveTrain.drive_RC(0,0,0)
        print("done with arcDrive")

