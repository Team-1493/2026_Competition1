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
        self.print_interval = 25
        self.numCams = 2   # number of cameras on robot



        self.driveTrain = DrivetrainGenerator.getInstance()
        self.headingController = HeadingController.getInstance()        
        self.constants =  ConstantValues.LimelightConstants

        self.max_value = 9999
        self.configfureLimelights()
        self.zeroAndseedIMU()
        self.cam_label = [" " for _ in range(self.numCams)]
        self.previous_estimate = [None for _ in range(self.numCams)]
        self.current_estimate = [PoseEstimate() for _ in range(self.numCams)]

        self.closestTagDist = [self.max_value]*2
        self.closestTagID = [0]*2
        self.closestAmb = [0]*2
        self.acceptEstimate = [False]*2
        self.stdXY = [self.max_value]*2
        self.stdRot = [self.max_value]*2

        SmartDashboard.putBoolean("Vision Active",True)
        for i in range(self.numCams):
                self.cam_label[i]="LL Cam "+str(i)+" "

        self.visionTimer = wpilib.Timer()
        self.set_id_filter_override(0,[2,3,4,5,8,9,10,11,18,19,20,21,24,25,26,27])
        self.set_id_filter_override(1,[2,3,4,5,8,9,10,11,18,19,20,21,24,25,26,27])    

        self.last_rot = 0    
        self.visionTimer.start()

        self.baseXY = self.constants.STD_DEV_COEFF_XY
        self.baseTheta = self.constants.STD_DEV_COEFF_THETA

        SmartDashboard.putNumber("X actual",0)
        SmartDashboard.putNumber("Y actual",0)


    def periodic(self):
       # Run vision at 50 Hz
        if self.visionTimer.advanceIfElapsed(0.05):
            self.currentPose = self.driveTrain.pose
            rot =  self.currentPose.rotation().degrees()
#            if True:
            if abs(rot-self.last_rot)>=1.0:
                for i in range(self.numCams):    
                    LimelightHelpers.set_robot_orientation(
                    self.constants.CAM_NAME[i],rot, 0, 0, 0, 0, 0)
                self.last_rot = rot                                   
            
            self.print_counter = self.print_counter+1
            
            self.update()


    def update(self):
#        previous_estimate = [None,None,None,None]
#        current_estimate = [PoseEstimate(),PoseEstimate(),PoseEstimate(),PoseEstimate()]                
        





                # read the pose estimate from each camera, and find the estimate
                # with either the closest individual tag or 
                # the min average tag distance
            
        for i in range(self.numCams):
#            SmartDashboard.putBoolean(self.cam_label[i]+" tv",
#                LimelightHelpers.get_tv(self.constants.CAM_NAME[i]))       

            self.current_estimate[i] = None

            self.previous_estimate[i],self.current_estimate[i] = self.pollLL(self.constants.CAM_NAME[i], 
                self.previous_estimate[i])    
            if self.current_estimate[i] is not None:
        
                for i in range(self.numCams):
                    self.acceptEstimate[i] = False   
                numTags = self.current_estimate[i].tag_count
                self.closestTagID[i],self.closestAmb[i],self.closestTagDist[i] = (
                    self.minDist(self.current_estimate[i].raw_fiducials))
                if numTags == 1:
                    t1 = self.currentPose.translation()
                    t2=self.current_estimate[i].pose.translation()
                    difference_check = t1.distance(t2)<self.constants.CAMERA_CUTOFF_DIFFERENCE
                    distance_check = self.closestTagDist[i]<self.constants.CAMERA_CUTOFF_DISTANCE_1
                        
                    if difference_check and distance_check: 
                        self.acceptEstimate[i] = True

                else:
                    distance_check = self.closestTagDist[i] < self.constants.CAMERA_CUTOFF_DISTANCE_2
                    if distance_check : 
                        self.acceptEstimate[i] = True

                if self.acceptEstimate[i]:
                    distance = self.closestTagDist[i]
                    omega = abs(self.driveTrain.get_omega_rps())
                    velocity = self.driveTrain.get_speeds_norm()
                    motionScale = 1 + 0.7*velocity + 0.5*omega
                    yawSpread = self.computeTagYawSpread(
                        self.current_estimate[i].raw_fiducials)
                    yawSpread = max(yawSpread, 1.0)
#                    print(yawSpread)

                    self.stdXY[i] = motionScale*self.baseXY * (distance ** 2) / math.sqrt(numTags)
                    self.stdRot[i] = motionScale*self.baseTheta * (distance ** 2) / (numTags*yawSpread)


                    if True:
                   # if SmartDashboard.getBoolean("Vision Active",True):
                     if self.closestAmb[i]<0.4:
                        x = self.current_estimate[i].pose.X()
                        y = self.current_estimate[i].pose.Y()
                        r = self.current_estimate[i].pose.rotation().radians()

                        self.driveTrain.add_vision_measurement(
                            self.current_estimate[i].pose,
# Use for PV !          utils.fpga_to_current_time(self.estimate.timestamp_seconds),
                            self.current_estimate[i].timestamp_seconds,
                        (self.stdXY[i], self.stdXY[i], self.stdRot[i]))


#                print(self.print_counter,"  ",self.print_interval)
                if(self.print_counter>self.print_interval):
                    label=self.cam_label[i]
                    SmartDashboard.putNumber(label+"closest Tag ID ",self.closestTagID[i])    
                    SmartDashboard.putNumber(label+"closest Dist",round(self.closestTagDist[i],3))                                    
                    SmartDashboard.putNumber(label+"Num Targ",self.current_estimate[i].tag_count)                
                    SmartDashboard.putNumber(label+"pose X",round(self.current_estimate[i].pose.translation().X(),3))
                    SmartDashboard.putNumber(label+"pose Y",round(self.current_estimate[i].pose.translation().Y(),3))
                    SmartDashboard.putNumber(label+"pose Rot",round(self.current_estimate[i].pose.rotation().degrees(),3) )                               
#                    SmartDashboard.putNumber(label+"pose X(in)",round(self.current_estimate[i].pose.translation().X()*39.37,3))
#                    SmartDashboard.putNumber(label+"pose Y(in)",round(self.current_estimate[i].pose.translation().Y()*39.37,3))
                    SmartDashboard.putNumber(label+"Amb",round(self.closestAmb[i],3))
#                    SmartDashboard.putNumber(label+"Dist avg",round(self.current_estimate[i].avg_tag_dist,3))                
#                   SmartDashboard.putNumber(label+"Area avg",round(current_estimate[i].avg_tag_area,3))    
#                   SmartDashboard.putNumber(label+"Pitch_ty",LimelightHelpers.get_ty(self.constants.CAM_NAME[i]))                                 
#                   SmartDashboard.putNumber(label+"Pitch_tync",LimelightHelpers.get_tync(self.constants.CAM_NAME[i]))
#                   SmartDashboard.putNumber(label+"Yaw_tx",LimelightHelpers.get_tx(self.constants.CAM_NAME[i]))                                 
#                   SmartDashboard.putNumber(label+"Yaw_txnc",LimelightHelpers.get_txnc(self.constants.CAM_NAME[i]))                
                    SmartDashboard.putNumber(label+"Std Dev XY",round(self.stdXY[i],3))  
                    SmartDashboard.putNumber(label+"Std Dev Theta",round(self.stdRot[i],3))  

            
        if self.print_counter>self.print_interval:
            self.print_counter=0

    def minDist(self,rf:List[RawFiducial]):
        minD=9999
        minID=0
        iMax=len(rf)
        i=0

        closest = min(rf, key=lambda t: t.dist_to_camera)
        return closest.id, closest.ambiguity, closest.dist_to_camera

    """
        while i<iMax:
            if rf[i].dist_to_camera<minD:
                minID=i
                minD=rf[i].dist_to_camera         
            i=i+1
        return rf[minID].id, rf[minID].ambiguity, minD,
    """

    def pollLL(self, id, previousEstimate: PoseEstimate):
        newEstimate = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(id)

        # Reject None immediately
        if newEstimate is None:
            return previousEstimate, None

        # Reject if no tags detected
        if newEstimate.tag_count <= 0:
            return previousEstimate, None

        # Get previous timestamp
        oldTimestamp = (
            previousEstimate.timestamp_seconds
            if previousEstimate is not None
            else self.max_value
        )

        # Reject duplicate timestamp
        if newEstimate.timestamp_seconds == oldTimestamp:
            return previousEstimate, None

        # Accept new estimate
        return newEstimate, newEstimate





    """"
    def pollLL(self,id,previousEstimate: PoseEstimate): 
        if (LimelightHelpers.get_tv(id)):
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
        
    """

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

    def computeTagYawSpread(self, rf:List[RawFiducial]):
        minYaw = float('inf')
        maxYaw = float('-inf')
        if len(rf) < 2:
            return 1.0

        for t in rf:
            y = t.txyc
            if y < minYaw: minYaw = y
            if y > maxYaw: maxYaw = y

        return maxYaw - minYaw



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

 