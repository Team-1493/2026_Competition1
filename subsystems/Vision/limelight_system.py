from typing import List
from commands2 import Subsystem
from phoenix6 import utils
from Utilities.LLH import LimelightHelpers
from Utilities.LLH import PoseEstimate
from Utilities.LLH import RawFiducial
from  Constants1 import ConstantValues
from wpilib import SmartDashboard

from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from subsystems.laser_can import LaserCAN

class LLsystem(Subsystem):
    instance = None

    @staticmethod
    def getInstance():
        if LLsystem.instance == None:
            LLsystem.instance = LLsystem()
            print("********************** LL System  **********************") 
        return LLsystem.instance
    

    def __init__(self):
        self.numCams = 1   # number of cameras on robot

 #       self.robotState = RobotState.getInstance()
        self.driveTrain = DrivetrainGenerator.getInstance()
        self.constants =  ConstantValues.LimelightConstants

        self.max_value = 9999

        self.closestTagDist = [self.max_value]*4
        self.closestTagID = [0]*4
        self.avgTagDist = [self.max_value]*4
        self.minClosestTagDist=self.max_value
        self.minAvgTagDist=self.max_value      
        self.camNum_minClosestTagDist=0
        self.camNum_minAvgTagDist=0
        self.canNum_best=-1
          

        self.previous_estimate = [None,None,None,None]
        self.current_estimate = [PoseEstimate(),PoseEstimate(),PoseEstimate(),PoseEstimate()]                
        self.estimate =  PoseEstimate()

        for i in range(self.numCams):
            self.closestTagDist[i] = 9999
            


        self.configfureLimelights()
        self.zeroAndseedIMU(0)


    def periodic(self):
        rot =  self.driveTrain.get_rotation_deg()
        for i in range(self.numCams):    
            LimelightHelpers.set_robot_orientation(
                self.constants.CAM_NAME[i],rot, 0, 0, 0, 0, 0)

        self.update()


    def update(self): 
        self.estimate = None
        self.minAvgTagDist = self.max_value
        self.minClosestTagDist = self.max_value
#        self.canNum_best=-1
        
        # check if we are moving too fast for an accurate camera measurement
        shouldAccept = (self.driveTrain.get_speeds_norm()<3 
            and abs(self.driveTrain.get_omega_rps())<2)
        SmartDashboard.putBoolean("LL accept",shouldAccept)

    


        if (shouldAccept):
            
            for i in range(self.numCams):

                cam_label="Cam"+str(i)
                self.closestTagDist[i] = None
                self.avgTagDist[i] = None            
                self.current_estimate[i]=None

                SmartDashboard.putBoolean(cam_label+" tv",
                    LimelightHelpers.get_tv(self.constants.CAM_NAME[i]))       

                # read the pose estimate from each camera, and find the estimate
                # with either the closest individual tag or 
                # the min average tag distance

                self.previous_estimate[i],self.current_estimate[i] = self.pollLL(self.constants.CAM_NAME[i], 
                    self.previous_estimate[i])
                

                if self.current_estimate[i] is not None and len(self.current_estimate[i].raw_fiducials)>0:     
                    #self.previous_estimate[i]=self.current_estimate[i]
                    self.avgTagDist[i] = (self.current_estimate[i].avg_tag_dist)


                    self.closestTagID[i],self.closestTagDist[i] = (
                        self.minDist(self.current_estimate[i].raw_fiducials))

                    if(self.avgTagDist[i]<self.minAvgTagDist):
                        self.minAvgTagDist=self.avgTagDist[i]
                        self.camNum_minAvgTagDist=i
                        # comment next 2 lines  to use minumum individual tag distance
                        #  self.estimate=self.current_estimate[i]
                        # self.canNum_best=i
                
                    if(self.closestTagDist[i]<self.minClosestTagDist):
                        self.minClosestTagDist=self.closestTagDist[i]
                        self.camNum_minClosestTagDist=i
                        # comment next 2 lines  to use min avg tag dfistance                     
                        self.estimate=self.current_estimate[i]
                        self.canNum_best=i
            
        
        
                    stdDev = self.constants.STD_DEV_COEFF_XY * (self.minClosestTagDist ** 2) / self.estimate.tag_count
                    #stdDev = 0.02 + 0.05 * self.minClosestTagDist
                    headingStdDev = self.constants.STD_DEV_COEFF_THETA * (self.minClosestTagDist** 2) / self.estimate.tag_count
                
                #  can change to avg Tag distance to use avg as cutoff criteria
                    if self.minClosestTagDist > self.constants.CAMERA_CUTOFF_DISTANCE:
                        stdDev = self.max_value
                        headingStdDev - self.max_value

                    """"                
                    self.driveTrain.add_vision_measurement(
                        self.estimate.pose,
#                        utils.fpga_to_current_time(self.estimate.timestamp_seconds),
                        self.estimate.timestamp_seconds,
                        (stdDev, stdDev, headingStdDev))
                    """


                    label="LL Cam"+str(i)+" "
                    SmartDashboard.putNumber(label+"closest Tag ID ",self.closestTagID[i])    
                    SmartDashboard.putNumber(label+"closest Dist",round(self.closestTagDist[i],3))                                    
                    SmartDashboard.putNumber(label+"Num Targ",self.current_estimate[i].tag_count)                
                    SmartDashboard.putNumber(label+"pose X",round(self.current_estimate[i].pose.translation().X(),3))
                    SmartDashboard.putNumber(label+"pose Y",round(self.current_estimate[i].pose.translation().Y(),3))
                    SmartDashboard.putNumber(label+"pose Rot",round(self.current_estimate[i].pose.rotation().degrees(),3) )                               
                    SmartDashboard.putNumber(label+"Dist avg",round(self.current_estimate[i].avg_tag_dist,3))                
                    SmartDashboard.putNumber(label+"Area avg",round(self.current_estimate[i].avg_tag_area,3))    
                    SmartDashboard.putNumber(label+"Pitch_ty",LimelightHelpers.get_ty(self.constants.CAM_NAME[i]))                                 
                    SmartDashboard.putNumber(label+"Pitch_tync",LimelightHelpers.get_tync(self.constants.CAM_NAME[i]))
                    SmartDashboard.putNumber(label+"Yaw_tx",LimelightHelpers.get_tx(self.constants.CAM_NAME[i]))                                 
                    SmartDashboard.putNumber(label+"Yaw_txnc",LimelightHelpers.get_txnc(self.constants.CAM_NAME[i]))                
                    SmartDashboard.putNumber(label+"Std Dev XY",round(stdDev,3))  
                    SmartDashboard.putNumber(label+"Std Dev Theta",round(headingStdDev,3))  


        SmartDashboard.putNumber("Best Cam",self.canNum_best)


    def minDist(self,rf:List[RawFiducial]):
        minD=9999
        minID=0
        iMax=len(rf)
        i=0

        while i<iMax:
            if rf[i].dist_to_camera<minD:
                minID=i
                minD=rf[i].dist_to_camera         
            i=i+1
        return rf[minID].id,minD




    def pollLL(self,id,previousEstimate: PoseEstimate): 
        if (LimelightHelpers.get_tv(id)):
            if previousEstimate is not None:

                oldTimestamp =  previousEstimate.timestamp_seconds 
            else:
                oldTimestamp = self.max_value
                
            newEstimate = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(id)
#            print("*********************   ",LimelightHelpers.get_tv(id),"  ",newEstimate.tag_count)
            
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
            rot=self.driveTrain.get_rotation_deg()  # LLH set_robot_orientation uses degrees

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
                        