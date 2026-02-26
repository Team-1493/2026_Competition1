import typing
from commands2 import Command
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState,Waypoint,IdealStartingState,RotationTarget,ConstraintsZone
from wpimath.kinematics import ChassisSpeeds;
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from subsystems.Drive.heading_controller import HeadingController
from Constants1 import ConstantValues 
import math
from math import hypot, pi
from Utilities.helper_methods import HelperMethods

from Commands.goal_pid import GoalPID
from Commands.goal_cam_command import GoalCamCommand



class DrivePathGenerator():
    def __init__(self,
                _robot: typing.Callable[[], Pose2d],
                 ) -> None:
        super().__init__()

        self.trench_right_pose = Pose2d(Translation2d(3.6,2.554),Rotation2d(0))
        self.trench_right_pose2 = Pose2d(Translation2d(3.6+2,2.554),Rotation2d(0))        
        self.trench_left_pose = Pose2d(Translation2d(3.6,8.069-2.554),Rotation2d(0))
        self.trench_left_pose2 = Pose2d(Translation2d(3.6+2,8.069-2.554),Rotation2d(0))        

        self.driveTrain = DrivetrainGenerator.getInstance()
        self.robot = _robot
        self.constantsVision = ConstantValues.getInstance().VisionConstants
        self.targetPose= Pose2d(Translation2d(3,3),Rotation2d(1.57))
        self.robotPose=self.robot()
        self.running=False
        self.path = None
        self.headingController = HeadingController.getInstance()
        self.constraints = PathConstraints(
            2.5, 
            3,
            2*pi,
            4*pi)


       

# Generate a commanmd to drive to a given pose using path-on-the-fly (create a path from the start and end poses)  
# Control the direction of travel at each waypiont usaing the the rotation of the poses used in wayppointsFromPoses
# Robot rotation controlled by GoalEndState
# Does not avoid feild elements
    def drive_path_to_pose(self,initialPose : Pose2d, _targetPose: Pose2d, maxVel,finalVel : float =None) -> Command:
        self.targetPose=_targetPose
        self.robotPose=self.robot()
        if finalVel is None:
            finalVel = 0

#             self.getPathVelocityHeading(self.getFieldVelocity(), self.targetPose))
        if initialPose==None:
            initialPose =Pose2d(self.robotPose.translation(),
                   self.getPathVelocityHeading(self.getFieldVelocity())) 
        listOfPoses = [initialPose, self.targetPose]
        self.waypoints = PathPlannerPath.waypointsFromPoses(listOfPoses) 

        constraints = PathConstraints(
            maxVel, 
            2*maxVel,
            2*pi,
            4*pi)


        iss =  IdealStartingState(
        self.getVelocityMagnitude(self.getFieldVelocity()), self.robotPose.rotation())
        rotTarget=[]
        # complete the rotatation at the halfway point so all rotation is done
        # well ahead of handover to the next stage
        rotTarget.append(RotationTarget(0.5,self.targetPose.rotation()))
        self.path =  PathPlannerPath(
            self.waypoints, 
            constraints,iss, 
            GoalEndState(finalVel, self.targetPose.rotation()),
            rotTarget
        )

        self.path.preventFlipping = True
        return ((AutoBuilder.followPath(self.path)))



# Generate a command to drive a path between two poses using PathFind.
# Will avoid field elements, but no control over direction of travel
    def drive_pathfind_to_pose(self,_targetPose, finalVel=None) -> Command:
        self.targetPose=_targetPose
        self.robotPose=self.robot()
        if finalVel is None:
            finalVel = 0
        return(AutoBuilder.pathfindToPose(self.targetPose, self.constraints,finalVel))           


    def getPathVelocityHeading(self,cs: ChassisSpeeds):
        if self.getVelocityMagnitude(cs) < 0.25:
            diff = Translation2d(self.targetPose.translation().X()-self.robotPose.translation().X(),
                              self.targetPose.translation().Y()-self.robotPose.translation().Y())                                      
            if diff.norm() < 0.01:
                return self.targetPose.rotation()
            else:
                return diff.angle()        
        else: return Rotation2d(cs.vx, cs.vy)
    

    def getVelocityMagnitude(self,cs:ChassisSpeeds):

        return hypot(cs.vx,cs.vy)
    

    """
        // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
        // but not the reverse.  However, because this transform is a simple rotation, negating the
        // angle given as the robot angle reverses the direction of rotation, and the conversion is
        // reversed.
    """

    def getFieldVelocity(self):
        robotRelativeSpeeds = self.driveTrain.kinematics.toChassisSpeeds(
            self.driveTrain.get_state().module_states)
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, 
                                                    self.robotPose.rotation())
    
    def waypoint_from_point(self,x,y):
        wp = Waypoint(Translation2d(x-0.3,y),Translation2d(x,y),None)
        if self.driveTrain.get_operator_forward_direction().degrees() == 180:
            wp=wp.flip()

# Get a command to drive a path-on-the-fly from robot position to a tag.
# Provide april Tag numnber, optoinal offset perpendicular to tag, offset parallel to tag

    def drive_path_to_tag(self,i:int, x_offset = 0, y_offset = 0):
        goalPose1 = HelperMethods.calculate_pose_goal_from_tag(i,x_offset-0.5,y_offset) 
        goalPose = HelperMethods.calculate_pose_goal_from_tag(i,x_offset,y_offset)

        return (
            (self.drive_path_to_pose(None,goalPose1,2.5, 1))  #speed of 1m/s at transition
#            .andThen(GoalCamCommand(x_offset,y_offset,i))
            .andThen(GoalPID(goalPose))
            .finallyDo(self.headingController.setTargetRotationInt))    
        

# Get a command to pathfind from robot position to a tag.
# Provide april Tag numnber, optoinal offset perpendicular to tag, offset parallel to tag
    def drive_pathfind_to_tag(self,i:int, x_offset = None, y_offset = None, velFinal = None):
        return self.drive_pathfind_to_pose(HelperMethods.calculate_pose_goal_from_tag(i,x_offset,y_offset),
                                        velFinal)

    def drive_trench(self):
        #need to add a check if the robot is in a corner and can't get to the trench position 
        targetpose=Pose2d()
        targetpose2=Pose2d()
#  NOT SYMETRIC!!!  If y<1/2 field width and blue, go to right trench, but if red go to left trench
#  This is because origin is always on blue side regardless of alliance color    
        if self.driveTrain.get_operator_forward_direction().degrees() == 0:
            if self.driveTrain.get_Y()>8.069/2:
                targetpose = self.trench_left_pose
                targetpose2 = self.trench_left_pose2
            else:
                targetpose = self.trench_right_pose
                targetpose2 = self.trench_right_pose2
        else:
            if self.driveTrain.get_Y()>8.069/2:
                targetpose = self.trench_right_pose
                targetpose2 = self.trench_right_pose2                
            else:
                targetpose = self.trench_left_pose
                targetpose2 = self.trench_left_pose2                            

        pose = HelperMethods.flip_pose_if_red(targetpose)
        pose2 = HelperMethods.flip_pose_if_red(targetpose2)

        return (
            (self.drive_path_to_pose(None,pose,2.5, .75))
            .andThen(self.drive_path_to_pose(pose,pose2,.75,0))  
            .finallyDo(self.headingController.setTargetRotationInt))    
        
  
