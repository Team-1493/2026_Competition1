from math import pi, cos,sin
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d
import math
import bisect
from Constants1 import ConstantValues
from pathplannerlib.path import Waypoint
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator

class HelperMethods():

    dt = DrivetrainGenerator.getInstance()
#    datadist = SmartDashboard.getNumberArray("Shoot Dist data",[0])
#    dataspd = SmartDashboard.getNumberArray("Shoot Speed data",[0])
#    data = [datadist,dataspd]

#    data=[[2.01,2.22,2.47,2.669,3.03,3.35,3.78,4.09],
#          [7.8,7.95,8.3,8.55,8.8,9.4,9.7,10.15]]

    datadist1 = [2.01,  2.22, 2.47, 2.67, 2.79,  3.03, 3.12, 3.34, 3.50, 3.78, 4.09]
    dataspd1 =  [8.10,  8.45, 8.68, 8.80, 8.89,  9.10, 9.25, 9.68, 9.72, 9.90, 10.1] 

    data = [datadist1,dataspd1]
    

    SmartDashboard.putNumberArray("Shoot Dist data",datadist1)
    SmartDashboard.putNumberArray("Shoot Speed data",dataspd1)


    """ calculates a goal pose from a BLUE tag ID, including an x and y offset from the tag face
     if alliance is red, the corresponding red tag is used!
    """ 
    @staticmethod
    def updateData():
        print("***************************************  DATA OLD:  ",HelperMethods.data[0][0],"  ",HelperMethods.data[0][1])        
       

        datadist1 = SmartDashboard.getNumberArray("Shoot Dist data",[0])
        dataspd1 = SmartDashboard.getNumberArray("Shoot Speed data",[0])
        HelperMethods.data = [datadist1,dataspd1]
        print("***************************************  DATA NEW:  ",HelperMethods.data[0][0],"  ",HelperMethods.data[0][1])        
    

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
    
    def calc_shoot_speed2():
        _, dist = HelperMethods.dist_to_hub()
        x, y = HelperMethods.data

        if dist <= x[0]:
            return y[0]
        if dist >= x[-1]:
            return y[-1]

        i = bisect.bisect_left(x, dist)

        x1, x2 = x[i-1], x[i]
        y1, y2 = y[i-1], y[i]
        t = (dist - x1) / (x2 - x1)
        return y1 + t * (y2 - y1)


    def calc_shoot_speed():

        angle,dist = HelperMethods.dist_to_hub()
        SmartDashboard.putNumber("Shooter hub dist",dist)
        data=HelperMethods.data
        c=dist
        s,t1=0,0
        for i in range(len(data[0])):
            if i ==0:
                s=data[0][i]-c
            if data[0][i]-c<=s:
                s=data[0][i]-c
                t1=i
        if s==0:return data[1][t1], m
        elif s<0:t2=t1-1
        elif s>0:t2=t1+1
        m=(data[1][t1]-data[1][t2])/(data[0][t1]-data[0][t2])#slope
        val = m*(c-data[0][t1])+data[1][t1]
        return val 

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