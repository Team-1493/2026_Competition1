from math import pi, cos,sin
from wpimath.geometry import Pose2d, Rotation2d
import math
from Constants1 import ConstantValues
from pathplannerlib.path import Waypoint
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator

class HelperMethods():
    dt = DrivetrainGenerator.getInstance()

    """ calculates a goal pose from a BLUE tag ID, including an x and y offset from the tag face
     if alliance is red, the corresponding red tag is used!
    """ 
    @staticmethod
    def calculate_pose_goal_from_tag(i : int, x_offset = 0, y_offset = 0):
        offset  = 0
        perspRot=HelperMethods.dt.get_operator_forward_direction().degrees()
        if perspRot==180:
            offset =16
        print("*****************  ",offset)
        poseTag = ConstantValues.VisionConstants.tags_list[i-1-offset].pose.toPose2d()
        rotTag = poseTag.rotation().radians()
        transTag = poseTag.translation()
        rotRobot=rotTag-pi

        poseGoal =  Pose2d(
            transTag.X()-x_offset*math.cos(rotTag)-y_offset*math.sin(rotTag),
            transTag.Y()-x_offset*math.sin(rotTag)+(y_offset)*math.cos(rotTag),
            Rotation2d(rotRobot))
        return (poseGoal)


    def flip_coordinates(x,y):
        perspRot =HelperMethods.dt.get_operator_forward_direction().degrees()
        if perspRot == 180:
            x=16.541-x
            y=8.069 - y
        return(x,y)
    

    def flip_waypoint(wp : Waypoint):
        perspRot =HelperMethods.dt.get_operator_forward_direction().degrees()
        if perspRot == 180:
            wp = wp.flip
        return(wp)
    
    def flip_pose_if_red(pose : Pose2d):
        pose_new = pose
        perspRot =HelperMethods.dt.get_operator_forward_direction().degrees()
        if perspRot == 180:
            pose_new = Pose2d(16.541-pose.X(),8.069 - pose.Y(),    
                pose.rotation() + Rotation2d.fromDegrees(180))
        return(pose_new)
