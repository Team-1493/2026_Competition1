import math
from typing import List
from commands2 import Subsystem
import wpilib
from Utilities.LLH import LimelightHelpers
from Utilities.LLH import PoseEstimate
from Utilities.LLH import RawFiducial
from  Constants1 import ConstantValues
from wpilib import SmartDashboard
from wpimath.geometry import Pose2d,Rotation2d,Translation2d

from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from subsystems.Drive.heading_controller import HeadingController



class LLsystem(Subsystem):
    instance = None

    @staticmethod
    def getInstance():
        if LLsystem.instance == None:
            LLsystem.instance = LLsystem()
            print("********************** LL System  **********************") 
        return LLsystem.instance


    def __init__(self):
        self.print_counter = 0
        self.print_interval = 10#10000
        self.numCams = 2   # number of cameras on robot



        self.driveTrain = DrivetrainGenerator.getInstance()
        self.headingController = HeadingController.getInstance()        
        self.constants =  ConstantValues.LimelightConstants

        self.max_value = 9999
        self.configfureLimelights()
        self.zeroAndseedIMU()
        self.cam_label = [" "," "]
        self.previous_estimate = [None,None]
        self.current_estimate = [PoseEstimate(),PoseEstimate()]                
        SmartDashboard.putBoolean("Vision Active",True)
        for i in range(self.numCams):
                self.cam_label[i]="LL Cam "+str(i)+" "

        self.visionTimer = wpilib.Timer()
        self.set_id_filter_override(0,[2,3,4,5,8,9,10,11,18,19,20,21,24,25,26,27])
        self.set_id_filter_override(1,[2,3,4,5,8,9,10,11,18,19,20,21,24,25,26,27])        
        
        self.closestTagDist = [self.max_value,self.max_value]
        self.closestTagID = [0,0]
        self.closestAmb = [0,0]
        self.acceptEstimate = [False,False]
        self.stdXY = [self.max_value,self.max_value]
        self.stdRot = [self.max_value,self.max_value]
        self.den = 1

        self.visionTimer.start()


        SmartDashboard.putNumber("X actual",0)
        SmartDashboard.putNumber("Y actual",0)


    def periodic(self):
       # Run vision at 50 Hz
        if self.visionTimer.advanceIfElapsed(0.02):
            self.currentPose = self.driveTrain.pose
            rot =  self.currentPose.rotation().degrees()
            for i in range(self.numCams):    
                LimelightHelpers.set_robot_orientation(
                   self.constants.CAM_NAME[i],rot, 0, 0, 0, 0, 0)
            self.print_counter = self.print_counter+1
            if self.print_counter>self.print_interval:
                self.print_counter=0

            self.update()


    def update(self):
        omega = abs(self.driveTrain.get_omega_rps()) 
        if omega<1.5:
        
            for i in range(self.numCams):

                self.current_estimate[i] = None

                self.previous_estimate[i],self.current_estimate[i] = self.pollLL(self.constants.CAM_NAME[i], 
                    self.previous_estimate[i])    
                

                if self.current_estimate[i] is not None :
                    numTags = self.current_estimate[i].tag_count

                    rf = self.current_estimate[i].raw_fiducials
                    if rf is None or len(rf) == 0:
                        continue

                    self.closestTagID[i],self.closestAmb[i],self.closestTagDist[i] = (
                        self.minDist(rf))
                
                    self.acceptEstimate[i] = False
                    distance = self.closestTagDist[i]
                    if numTags==1:
                        if self.closestAmb[i]<0.2:
                            if distance <self.constants.CAMERA_CUTOFF_DISTANCE_1:
                                t1 = self.currentPose.translation()
                                t2=self.current_estimate[i].pose.translation()
                                if t1.distance(t2)<self.constants.CAMERA_CUTOFF_DIFFERENCE:
                                    self.acceptEstimate[i] = True
                    else:
                        if self.closestAmb[i]<0.3:
                            if distance < self.constants.CAMERA_CUTOFF_DISTANCE_2:
                                self.acceptEstimate[i] = True

                    if self.acceptEstimate[i]:
                        velocity = self.driveTrain.get_speeds_norm()
                        motionScale = 1 + 0.7*velocity + 0.5*omega

                        baseXY = self.constants.STD_DEV_COEFF_XY
                        baseTheta = self.constants.STD_DEV_COEFF_THETA                       
                        d2 = distance*distance
                        den = 1 if numTags == 1 else 1.4                        
                        self.stdXY[i] = motionScale*baseXY * d2 / den
                        self.stdRot[i] =motionScale*baseTheta * d2 / numTags
                    
                        self.driveTrain.add_vision_measurement(
                                self.current_estimate[i].pose,
                                self.current_estimate[i].timestamp_seconds,
                            (self.stdXY[i], self.stdXY[i], self.stdRot[i]))


#                print(self.print_counter,"  ",self.print_interval)
                    if(self.print_counter==self.print_interval):
                        label=self.cam_label[i]
                        SmartDashboard.putNumber(label+"closest Tag ID ",self.closestTagID[i])    
                        SmartDashboard.putNumber(label+"closest Dist",round(self.closestTagDist[i],3))                                    
                        SmartDashboard.putNumber(label+"Num Targ",self.current_estimate[i].tag_count)                
                        SmartDashboard.putNumber(label+"pose X",round(self.current_estimate[i].pose.translation().X(),3))
                        SmartDashboard.putNumber(label+"pose Y",round(self.current_estimate[i].pose.translation().Y(),3))
                        SmartDashboard.putNumber(label+"pose Rot",round(self.current_estimate[i].pose.rotation().degrees(),3) )                               
                        SmartDashboard.putNumber(label+"pose X(in)",round(self.current_estimate[i].pose.translation().X()*39.37,3))
                        SmartDashboard.putNumber(label+"pose Y(in)",round(self.current_estimate[i].pose.translation().Y()*39.37,3))
                        SmartDashboard.putNumber(label+"Amb",round(self.closestAmb[i],3))
                        SmartDashboard.putNumber(label+"Std Dev XY",round(self.stdXY[i],3))  
                        SmartDashboard.putNumber(label+"Std Dev Theta",round(self.stdRot[i],3))  

            

    def minDist(self,rf:List[RawFiducial]):
        minD=9999
        minID=0
        iMax=len(rf)
        i=0
        amb=999

        while i<iMax:
            if rf[i].dist_to_camera<minD:
                minID=i
                minD=rf[i].dist_to_camera         
            i=i+1
        return rf[minID].id, rf[minID].ambiguity, minD,



    def pollLL(self,id,previousEstimate: PoseEstimate): 
        if previousEstimate is not None:
            oldTimestamp =  previousEstimate.timestamp_seconds 
        else:
            oldTimestamp = self.max_value
                
        newEstimate = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(id)
            
        if newEstimate is not None and newEstimate.tag_count>0:
            if newEstimate.timestamp_seconds == oldTimestamp:
                newEstimate = None
            else:
                previousEstimate = newEstimate
        else:
            newEstimate = None
        return previousEstimate,newEstimate         
        


    def configfureLimelights(self):
        for i in range(self.numCams):
            LimelightHelpers.set_camerapose_robotspace(
                self.constants.CAM_NAME[i],
                self.constants.CAM_X_OFFSET[i], 
                self.constants.CAM_Y_OFFSET[i],
                self.constants.CAM_Z_OFFSET[i],
                self.constants.CAM_THETA_X_OFFSET[i],
                self.constants.CAM_THETA_Y_OFFSET[i],
                self.constants.CAM_THETA_Z_OFFSET[i])
            


    
    def zeroAndseedIMU(self,rot=None):
        if rot is None:
            rot = self.driveTrain.pose.rotation().degrees()  # LLH set_robot_orientation uses degrees

        for i in range(self.numCams):
            # send the current robot pose to the limelight
            LimelightHelpers.set_robot_orientation(self.constants.CAM_NAME[i], rot, 0, 0, 0, 0, 0)
            # use external IMU, seed internal IMU with value from set_robot_orientation
            LimelightHelpers.set_imu_mode(self.constants.CAM_NAME[i], 1)


    def set_IMU_Mode(self, mode:int):
        for i in range(self.numCams):
            LimelightHelpers.set_imu_mode(self.constants.CAM_NAME[i], mode)   

# only these tags can be detected, limits what tags used for megatag
# do this on a specified camera since it mayu differ by camera
    def set_id_filter_override(self,cam_number,idList:List[int]):
        LimelightHelpers.set_fiducial_id_filters_override(self.constants.CAM_NAME[cam_number],idList)

# choose preferred tag for best target, determines which tag used for tx, ty, ta
# megatag still uses all visible tags
# do this on a specified camera since it mayu differ by camera
    def set_priority_tag(self,cam_number,id):
        LimelightHelpers.set_priority_tag_id(self.constants.CAM_NAME[cam_number],id)     


    def write_camera0_pose_to_file(self):
        if self.numCams < 1:
            return

        cam_name = self.constants.CAM_NAME[0]
        estimate = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(cam_name)

        if estimate is None or estimate.tag_count <= 0:
            return

        closest_tag_id = 0
        closest_tag_distance = 0.0
        if estimate.raw_fiducials is not None and len(estimate.raw_fiducials) > 0:
            closest_tag_id, closest_amb,closest_tag_distance = self.minDist(estimate.raw_fiducials)

        pose_x = estimate.pose.translation().X()*39.37  
        pose_y = estimate.pose.translation().Y()*39.37
        pose_rot = estimate.pose.rotation().degrees()
        x_act = SmartDashboard.getNumber("X actual", 0)*12
        y_act = SmartDashboard.getNumber("Y actual", 0)*12

        with open("/home/lvuser/limelight_camera0_pose_log.txt", "a", encoding="utf-8") as pose_file:
#        with open("limelight_camera0_pose_log.txt", "a", encoding="utf-8") as pose_file:
            pose_file.write(
                f"{x_act:.3f}\t{y_act:.3f}\t{pose_x:.3f}\t{pose_y:.3f}\t"
                f"{pose_rot:.3f}\t{closest_tag_id}\t{closest_tag_distance:.3f}\t"
                f"{estimate.tag_count}\n"
            )

 