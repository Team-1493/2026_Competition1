from math import pi, cos,sin
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d
import math
from Constants1 import ConstantValues
from pathplannerlib.path import Waypoint
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator

class HelperMethods():
    dt = DrivetrainGenerator.getInstance()
    data=[[2.49,2.861,3.17,3.45,3.62,3.91,4.1,4.4],
          [8.0,8.45,8.86,9.10,9.45,9.90,10.0,10.25]]

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

    def calculate_shoot_speed():
        angle,dist = HelperMethods.dist_to_hub()
        SmartDashboard.putNumber("Shooter hub dist",dist)
#        return ((dist*39.37)*.0337+4.65)
        return (1.245*dist+4.912)   
    
    def calc_shoot_speed():
       
        angle,dist = HelperMethods.dist_to_hub()
        SmartDashboard.putNumber("Shooter hub dist",dist)
        #SmartDashboard.putNumber("m",m)
        data=HelperMethods.data
        c=dist
        s,t1=0,0
        for i in range(len(data[0])):
            if i ==0:
                s=data[0][i]-c
            if data[0][i]-c<=s:
                s=data[0][i]-c
                t1=i
        if s==0:
            return data[1][t1], m
        elif s<0:
            t2=t1-1
        elif s>0:
            t2=t1+1
        m=(data[1][t1]-data[1][t2])/(data[0][t1]-data[0][t2])#slope
        val = m*(c-data[0][t1])+data[1][t1]
        return val

    def calc_shoot_speed2():
        _,d=HelperMethods.dis_to_hub()
        data = [[1,2,3,4,5,6],[3,4,5,6,7,8]]
        c=1.5#test number
        closest_two = sorted(data[0], key=lambda x: abs(x-c))[:2]

        

        return 

    def dist_to_hub():
        pose = HelperMethods.dt.pose
        xr = pose.X()
        yr = pose.Y()
        
        color = HelperMethods.dt.alliance_color
        if color == None: color = DriverStation.Alliance.kBlue
        if color==DriverStation.Alliance.kBlue:
            xh = 4.644
            yh= 4.030

        else:
            xh = 11.92
            yh= 4.041

        xr1=(xh - xr)
        yr1 = yh - yr

        xt=-yr1
        yt=xr1

        angle_hub=math.atan2(yt,xt)-math.pi/2  
        dist_hub=math.hypot(xt,yt) 
        SmartDashboard.putNumber("Distance To Hub",dist_hub)        
        return angle_hub,dist_hub