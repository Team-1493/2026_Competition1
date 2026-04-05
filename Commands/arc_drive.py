import math
from typing import override
import typing
import commands2
from wpilib import DriverStation, SmartDashboard
from wpimath import units
from wpimath.geometry import Translation2d
from Commands.stop_drive import StopDrive
from subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from phoenix6 import swerve
from wpimath.geometry import Pose2d,Rotation2d
from Commands.drive_path_generator import DrivePathGenerator
from pathplannerlib.config  import PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController 
from pathplannerlib.trajectory import PathPlannerTrajectoryState
from wpimath.kinematics import ChassisSpeeds



class arcDrive(commands2.Command):
    def __init__(self,
                _driveTrain:CommandSwerveDrivetrain
                # _targetPose: typing.Callable[[], Pose2d],
                ) -> None:
        super().__init__()
        self.driveTrain = _driveTrain
        self.max_radius = 4
        self.min_radius = 2

        self.max_speed=2.0 #max speed
        self.max_omega=4  # max rotation rate
        self.is_near_target = False
        self.is_at_target = False        
        self.nearTarget_dist_tol = 0.152 # 6 inche
        self.nearTarget_angle_tol = 0.052 # 3 degrees
        self.atTarget_dist_tol = 0.051 # 2 inches
        self.atTarget_angle_tol = 0.026 # 1.5 degree
        self.kTranslationPID = PIDConstants(3.0,0)
        self.kRotationPID = PIDConstants(3,0,0)
        self.controller = PPHolonomicDriveController(
            self.kTranslationPID, self.kRotationPID)
        self.addRequirements(self.driveTrain)


    @override
    def initialize(self):
        currentPose = self.driveTrain.pose
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


        hub_trans=Translation2d(self.xh,self.yh)
        distance_to_hub = hub_trans.distance(currentPose.translation())

        if (distance_to_hub<=self.max_radius) and (distance_to_hub>=self.min_radius):
            self.y_goal=yr
            self.x_goal=xr
        else:
            if distance_to_hub>self.max_radius:
                radius = self.max_radius
            else:
                radius = self.min_radius
            xt_goal=radius*xr1/math.sqrt(xr1*xr1 + yr1*yr1)
            yt_goal=radius*yr1/math.sqrt(xr1*xr1 + yr1*yr1)
            self.y_goal=self.yh - yt_goal
            self.x_goal=self.xh -xt_goal

        self.angle_goal=math.atan2(yt,xt)-math.pi/2  

        self.goalPose = Pose2d(self.x_goal,self.y_goal,Rotation2d(self.angle_goal)) 
        self.goalState = PathPlannerTrajectoryState()
        self.goalState.pose = self.goalPose

    def execute(self):
        speeds =self.controller.calculateRobotRelativeSpeeds(
            self.driveTrain.pose, self.goalState)
        
        speed_mag = math.hypot(speeds.vx, speeds.vy)
        omega = speeds.omega
        
        if omega>0:
            omega = min(omega,self.max_omega)
        elif omega<0:
            omega=max(omega,-self.max_omega)
        
        if speed_mag > self.max_speed:
            scale = self.max_speed / speed_mag
            speeds = ChassisSpeeds(speeds.vx * scale,speeds.vy * scale, omega)
        else:
            speeds = ChassisSpeeds(speeds.vx,speeds.vy,omega)    
        print(omega)        
#        print(speeds.vx,"  ",speeds.vy,"  ",speeds.omega,"  ",self.driveTrain.pose.rotation().degrees())
        if self.validLocation:
            self.driveTrain.set_control(
                self.driveTrain.request_autogenerator.with_speeds(speeds))
        self.calculate_error()

    @override
    def isFinished(self):
        if self.validLocation is False:
            return True
        return self.is_at_target

  

    @override
    def end(self,interrupted:bool):
        print("DONE WITH ARCDRIVE !!!!!!")
        self.driveTrain.drive_RC(0,0,0)

    def calculate_error(self):
        current = self.driveTrain.pose
        xyErr =math.hypot(current.X() - self.x_goal,current.Y() - self.y_goal) 
        near_xy = xyErr <= self.nearTarget_dist_tol
        at_xy =  xyErr <= self.atTarget_dist_tol

        thetaErr = abs(current.rotation().radians() - self.angle_goal)
        if thetaErr>math.pi: thetaErr = thetaErr - 2*math.pi
        if thetaErr<-math.pi: thetaErr = thetaErr - 2*math.pi 
        near_angle = abs(thetaErr) <= self.nearTarget_angle_tol
        at_angle = abs(thetaErr) <= self.atTarget_angle_tol

        print(xyErr,"   ",thetaErr,"  ",self.is_near_target,"  ",self.is_at_target)
        self.is_at_target = at_angle and at_xy
        self.is_near_target  = near_angle and near_xy


    def get_is_near(self):
        return self.is_near_target
