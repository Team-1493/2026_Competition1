import math
from typing import List
from commands2 import Subsystem
import wpilib
from Utilities.LLH import LimelightHelpers
from Utilities.LLH import PoseEstimate
from Utilities.LLH import RawFiducial
from  Constants1 import ConstantValues
from wpilib import SmartDashboard

from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from subsystems.Drive.heading_controller import HeadingController

from datetime import datetime




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
        self.print_interval = 0
        self.numCams = 2   # number of cameras on robot



        self.driveTrain = DrivetrainGenerator.getInstance()
        self.headingController = HeadingController.getInstance()        
        self.constants =  ConstantValues.LimelightConstants

        self.max_value = 9999
        self.configfureLimelights()
        self.zeroAndseedIMU(0)
        self.cam_label = [" "]*self.numCams
        self.previous_estimate = [None]*self.numCams
        self.current_estimate = [PoseEstimate()]*self.numCams                
        SmartDashboard.putBoolean("Vision Active",False)
        for i in range(self.numCams):
                self.cam_label[i]="LL Cam "+str(i)+" "

        self.visionTimer = wpilib.Timer()
        self.visionTimer.start()


        SmartDashboard.putNumber("X actual",0)
        SmartDashboard.putNumber("Y actual",0)


    def periodic(self):
       # Run vision at 20 Hz
        if self.visionTimer.advanceIfElapsed(0.05):
            self.currentPose = self.driveTrain.pose
#           rot =  self.currentPose.rotation().degrees()+self.headingController.rotation_offset*math.pi/180.
            rot =  self.currentPose.rotation().degrees()
            for i in range(self.numCams):    
                LimelightHelpers.set_robot_orientation(
                   self.constants.CAM_NAME[i],rot, 0, 0, 0, 0, 0)
            self.print_counter = self.print_counter+1
            self.update()


    def update(self):
#        previous_estimate = [None,None,None,None]
#        current_estimate = [PoseEstimate(),PoseEstimate(),PoseEstimate(),PoseEstimate()]                
        estimate = PoseEstimate()
        
        closestTagDist = [self.max_value]*4
        closestTagID = [0]*4
        acceptEstimate = [False]*4
        stdXY = [self.max_value]*4
        stdRot = [self.max_value]*4




                # read the pose estimate from each camera, and find the estimate
                # with either the closest individual tag or 
                # the min average tag distance
            
        for i in range(self.numCams):
            SmartDashboard.putBoolean(self.cam_label[i]+" tv",
                LimelightHelpers.get_tv(self.constants.CAM_NAME[i]))       

            self.current_estimate[i] = None

            self.previous_estimate[i],self.current_estimate[i] = self.pollLL(self.constants.CAM_NAME[i], 
                self.previous_estimate[i])    
            if self.current_estimate[i] is not None and self.current_estimate[i].tag_count>0:
                numTags = self.current_estimate[i].tag_count
                closestTagID[i],closestTagDist[i] = (
                    self.minDist(self.current_estimate[i].raw_fiducials))
                if numTags == 1:
                    t1 = self.currentPose.translation()
                    t2=self.current_estimate[i].pose.translation()
                    difference_check = t1.distance(t2)<self.constants.CAMERA_CUTOFF_DIFFERENCE
                    distance_check = closestTagDist[i]<self.constants.CAMERA_CUTOFF_DISTANCE_1
                        
                    if difference_check and distance_check: 
                        acceptEstimate[i] = True

                else:
                    distance_check = closestTagDist[i] < self.constants.CAMERA_CUTOFF_DISTANCE_2
                    if distance_check : 
                        acceptEstimate[i] = True

                if acceptEstimate[i]:
                    distance = closestTagDist[i]
                    numTags = self.current_estimate[i].tag_count
                    baseXY = self.constants.STD_DEV_COEFF_XY
                    baseTheta = self.constants.STD_DEV_COEFF_THETA
                    omega = abs(self.driveTrain.get_omega_rps())
                    velocity = self.driveTrain.get_speeds_norm()
                    motionScale = 1 + 0.7*velocity + 0.5*omega
                    yawSpread = self.computeTagYawSpread(
                        self.current_estimate[i].raw_fiducials)
                    yawSpread = max(yawSpread, 1.0)
#                    print(yawSpread)

                    stdXY[i] = motionScale*baseXY * (distance ** 2) / math.sqrt(numTags)
                    stdRot[i] = motionScale*baseTheta * (distance ** 2) / (numTags*yawSpread)



                    if SmartDashboard.getBoolean("Vision Active",True):
                        self.driveTrain.add_vision_measurement(
                            self.current_estimate[i].pose,
# Use for PV !          utils.fpga_to_current_time(self.estimate.timestamp_seconds),
                            self.current_estimate[i].timestamp_seconds,
                        (stdXY[i], stdXY[i], stdRot[i]))


#                print(self.print_counter,"  ",self.print_interval)
                if(self.print_counter>self.print_interval):
                    label=self.cam_label[i]
                    SmartDashboard.putNumber(label+"closest Tag ID ",closestTagID[i])    
                    SmartDashboard.putNumber(label+"closest Dist",round(closestTagDist[i],3))                                    
                    SmartDashboard.putNumber(label+"Num Targ",self.current_estimate[i].tag_count)                
                    SmartDashboard.putNumber(label+"pose X",round(self.current_estimate[i].pose.translation().X(),3))
                    SmartDashboard.putNumber(label+"pose Y",round(self.current_estimate[i].pose.translation().Y(),3))
                    SmartDashboard.putNumber(label+"pose Rot",round(self.current_estimate[i].pose.rotation().degrees(),3) )                               
                    SmartDashboard.putNumber(label+"pose X(in)",round(self.current_estimate[i].pose.translation().X()*39.37,3))
                    SmartDashboard.putNumber(label+"pose Y(in)",round(self.current_estimate[i].pose.translation().Y()*39.37,3))

                    SmartDashboard.putNumber(label+"Dist avg",round(self.current_estimate[i].avg_tag_dist,3))                
#                   SmartDashboard.putNumber(label+"Area avg",round(current_estimate[i].avg_tag_area,3))    
#                   SmartDashboard.putNumber(label+"Pitch_ty",LimelightHelpers.get_ty(self.constants.CAM_NAME[i]))                                 
#                   SmartDashboard.putNumber(label+"Pitch_tync",LimelightHelpers.get_tync(self.constants.CAM_NAME[i]))
#                   SmartDashboard.putNumber(label+"Yaw_tx",LimelightHelpers.get_tx(self.constants.CAM_NAME[i]))                                 
#                   SmartDashboard.putNumber(label+"Yaw_txnc",LimelightHelpers.get_txnc(self.constants.CAM_NAME[i]))                
                    SmartDashboard.putNumber(label+"Std Dev XY",round(stdXY[i],3))  
                    SmartDashboard.putNumber(label+"Std Dev Theta",round(stdRot[i],3))  

            
        if self.print_counter>self.print_interval:
            self.print_counter=0

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
            rot=self.driveTrain.rotation_deg  # LLH set_robot_orientation uses degrees

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
        if len(rf) < 2:
            return 1.0

        yaws = [t.txyc for t in rf]  # normalized yaw
        return max(yaws) - min(yaws)

    def ambiguity_distance_primary(self,name,rawFiducials: List[RawFiducial], primary_id: int):
        primary_id = LimelightHelpers.get_fiducial_id(name)
        primary_ambiguity = None

        for f in rawFiducials:
            if f.id == primary_id:
                primary_ambiguity = f.ambiguity
                primary_dist = f.dist_to_camera
                return primary_ambiguity, primary_dist
        return None

    def primary_tag_pose(self,id):

        tagPoses = [
            [183.19, 132.69, 46.32, 0, -90.0, 0],
            [184.80, 178.98, 46.44, 0, 90.0, 0],
            [181.65, 288.90, 36.51, 0, -179.2, 0],
            [170.67, 179.45, 46.37, 0, 87.7, 0],
            [159.88, 170.24, 46.50, 0, 176.6, 0],
            [159.37, 156.26, 46.33, 0, 179.1, 0],
            [169.22, 132.65, 46.36, 0, -88.2, 0],
            [182.13, 23.15, 36.24, 0, 179.3, 0],
            [0.31, 26.22, 21.75, 0, 0.0, 0],
            [0.51, 40.91, 21.82, 0, -1.1, 0],
            [4.21, 137.32, 23.76, 0, -0.4, 0],
            [4.76, 154.69, 23.64, 0, -0.7, 0]
            ]
        tagIDs = [18, 21, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32]
        return tagPoses[tagIDs.index(id)]


    def write_camera0_pose_to_file(self):
        if self.numCams < 1:
            return
#        now = datetime.now()
#        timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
    # Filename A used to store cam-tag pose and robot odometry pose
    # for calibrating cam-robot transform.
    # Filename B used to store robot odometry pose and megatag pose for verifacation
    # of limelight calibrations
 #       filenameA = f"LLcalA_{timestamp}.txt"
 #       filenameB = f"LLcalA_{timestamp}.txt"        

        filenameA = "/home/lvuser/LLcallibrationData_A.txt"
        filenameB = "/home/lvuser/LLcallibrationData_B.txt"        


        estimate = None
        cam_name = self.constants.CAM_NAME[0]
        cam_to_target = LimelightHelpers.get_camerapose_targetspace(cam_name)
        primary_id = LimelightHelpers.get_fiducial_id(cam_name)
        estimate = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(cam_name)
        robotPose = self.driveTrain.pose


        if primary_id<1 or estimate is None or estimate.tag_count<1:
            return

      
        cam_target_x = cam_to_target[0]  
        cam_target_y = cam_to_target[1]
        cam_target_z = cam_to_target[2]
        cam_target_pitch = cam_to_target[3]*math.pi/180.0
        cam_target_yaw = cam_to_target[4]*math.pi/180.0
        cam_target_roll = cam_to_target[5]*math.pi/180.0

        pose_tag = self.primary_tag_pose(primary_id)
        tag_x = pose_tag[0] 
        tag_y = pose_tag[1]
        tag_z = pose_tag[2]
        tag_pitch = pose_tag[3]*math.pi/180.0
        tag_yaw = pose_tag[4]*math.pi/180.0
        tag_roll = pose_tag[5]*math.pi/180.0

        mega_x = estimate.pose.translation().X()
        mega_y = estimate.pose.translation().Y()    
        mega_thetaDegrees = estimate.pose.rotation().degrees()
        mega_tag_count = estimate.tag_count
        primary_ambiguity, primary_dist = self.ambiguity_distance_primary(cam_name,estimate.raw_fiducials, primary_id)

        poseRobot_x = robotPose.X()
        poseRobot_y = robotPose.Y()
        poseRobot_theta = robotPose.rotation().radians()
        poseRobot_thetaDegrees = robotPose.rotation().degrees()        

        with open(filenameA, "a", encoding="utf-8") as fileA:
            fileA.write(
                f"{cam_target_x:.3f}\t{cam_target_y:.3f}\t{cam_target_z:.3f}\t{cam_target_pitch:.3f}\t{cam_target_yaw:.3f}\t{cam_target_roll:.3f}\t"                
                f"{tag_x/39.37:.3f}\t{tag_y/39.37:.3f}\t{tag_z/39.37:.3f}\t{tag_pitch:.3f}\t{tag_yaw:.3f}\t{tag_roll:.3f}\t"
                f"{poseRobot_x:.3f}\t{poseRobot_y:.3f}\t{poseRobot_theta:.3f}\t"
                f"{primary_id}\t{primary_ambiguity:.3f}\t{primary_dist:.3f}\t{mega_tag_count}\n"
            )

        with open(filenameB, "a", encoding="utf-8") as fileB:
            fileB.write(
                f"{poseRobot_x:.3f}\t{poseRobot_y:.3f}\t{poseRobot_thetaDegrees:.3f}\t"
                f"{mega_x:.3f}\t{mega_y:.3f}\t{mega_thetaDegrees:.3f}\t"
                f"{primary_ambiguity:.3f}\t{primary_dist:.3f}\n"
            ) 