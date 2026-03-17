from robotpy_apriltag import AprilTagFieldLayout

from commands2 import Subsystem
from phoenix6  import utils
from ntcore import NetworkTable, NetworkTableEntry, NetworkTableInstance, DoubleArrayEntry
import photonlibpy 
from photonlibpy import EstimatedRobotPose, PhotonPoseEstimator

from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.simulation import VisionSystemSim, PhotonCameraSim,SimCameraProperties
from Utilities.LLH import LimelightHelpers
from wpilib import RobotBase
from wpimath.geometry import Rotation2d,Translation3d,Rotation3d, Transform3d
from Constants1 import ConstantValues
from math import radians
from wpilib import SmartDashboard
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator

class PVisionSim(Subsystem):
    
    def __init__(self):

#    def simulationInit(self):

 #    if RobotBase.isSimulation():
    
#            self.layout = ConstantValues.VisionConstants.field_layout


# Load the field layout
 

#            self.robotState = RobotState.getInstance() 
            self.driveTrain = DrivetrainGenerator.getInstance()
            self.visionSim = VisionSystemSim("PVsim")
            self.layout = ConstantValues.VisionConstants.field_layout 
            self.visionSim.addAprilTags(self.layout)  
            self.dummyCam = PhotonCamera("dummyCam")
            
            cameraProp = SimCameraProperties()
            cameraProp.setCalibrationFromFOV(960, 720, Rotation2d.fromDegrees(90))
            cameraProp.setCalibError(.15, 0.05)#0.35,0.10 default
            cameraProp.setFPS(50)
            cameraProp.setAvgLatency(0.035)
            cameraProp.setLatencyStdDev(.005)
            

            self.cameraSim = PhotonCameraSim(self.dummyCam, cameraProp,self.layout,0,4)
       #     self.cameraSim.enableRawStream(True)
#            self.cameraSim.enableProcessedStream(True)
            
            robotToCameraTrl = Translation3d(0, 0, 1)  
            robotToCameraRot = Rotation3d(0, radians(0), 0) #pitched 15 degrees up
            robotToCamera =  Transform3d(robotToCameraTrl, robotToCameraRot)

            self.visionSim.addCamera(self.cameraSim, robotToCamera)
            self.visionSim.getDebugField()

            self.camName = ConstantValues.LimelightConstants.CAM_NAME[0]
            

            self.photonEstimator = PhotonPoseEstimator(self.layout, robotToCamera)
            
#            self.photonEstimator.multiTagFallbackStrategy=PoseStrategy.LOWEST_AMBIGUITY
            # is there a visible target in the currently read results, set to zero if no new result!           
            self.targetVisible = False   
            # does the camera see a target, value doesn't change if no new result !            
            self.hasTarget = False 

    def simulationPeriodic(self):
       if RobotBase.isSimulation():
            estimatedRobotPose = None

            self.visionSim.update(self.driveTrain.get_pose())
            mindist=999
            results = self.dummyCam.getAllUnreadResults()
            length=len(results)
            self.targetVisible = False
            if len(results) > 0:
                self.hasTarget = False
                result = results[-1]  # take the most recent result the camera had
                length=len(result.getTargets())
                rawFid = []
                avg_dist=0
                avg_area = 0
                for target in result.getTargets():
                    self.targetVisible=True
                    self.hasTarget = True
                    dist = target.getBestCameraToTarget().translation().norm()
                    area = target.getArea()
                    avg_dist=avg_dist+dist
                    avg_area=avg_area+area                    
                    rawFid.append(target.getFiducialId())   # To Do - rewrite more concise 
                    rawFid.append(target.getYaw())  #txnc
                    rawFid.append(target.getPitch())  #tync
                    rawFid.append(target.getArea())
                    rawFid.append(dist)  # dist to Cam
                    rawFid.append(dist)  # dist to robot
                    rawFid.append(target.getPoseAmbiguity())
                    if dist<mindist:
                          mindist=dist
                          minPitch = target.getPitch()
                          minYaw = target.getYaw()                          
                          minID=target.getFiducialId()
                          minArea = target.getArea()
                
                if length>0: 
                    avg_dist=avg_dist/length
                    avg_area=avg_area/length
            
            if self.targetVisible:
                estimatedRobotPose = self.photonEstimator.estimateCoprocMultiTagPose(result)
#                print("PV pose is none ",estimatedRobotPose is None)
            SmartDashboard.putNumber("PV hasTarget",self.hasTarget)
            LimelightHelpers.set_tv(self.camName, self.hasTarget)


            if (length>0 and not self.targetVisible) : 
                SmartDashboard.putNumber("PV ID near",0)
                SmartDashboard.putNumber("PV Dist near ",0)     
                SmartDashboard.putNumber("PV num Targets ",0)                                
            elif length>0 and self.targetVisible:
                SmartDashboard.putNumber("PV ID near ",minID)
                SmartDashboard.putNumber("PV Dist near ",mindist) 
                SmartDashboard.putNumber("PV pitch near ",minPitch)                    
                SmartDashboard.putNumber("PV Yaw near ",minYaw)   
                SmartDashboard.putNumber("PV Area avg ",avg_area)                   
                SmartDashboard.putNumber("PV Area near ",minArea)                                   
                SmartDashboard.putNumber("PV num Targets ",length)                                                     
            
            if (estimatedRobotPose is not None) :
# Put this back in if using PV for pose correction, comment out if using PV to drive LL in simulation                
                
                if avg_dist<300:
                    estPose =estimatedRobotPose.estimatedPose.toPose2d() 
                    stdDev =    (0.3, 0.3, 999)
                    """                         

                    self.driveTrain.add_vision_measurement(
                        estPose,
                        utils.fpga_to_current_time(estimatedRobotPose.timestampSeconds), stdDev)
                    """

                    SmartDashboard.putNumber("PV pose X",estPose.translation().X())
                    SmartDashboard.putNumber("PV pose Y",estPose.translation().Y()) 
                    SmartDashboard.putNumber("PV pose R",estPose.rotation().radians() )                               
                    SmartDashboard.putNumber("PV Dist avg",avg_dist)    
                
                LimelightHelpers.set_ty(self.camName, minPitch) 
                LimelightHelpers.set_tx(self.camName, minYaw)                 
                LimelightHelpers.set_tync(self.camName, minPitch) 
                LimelightHelpers.set_txnc(self.camName, minYaw)  
                LimelightHelpers.set_fiducialid_id(self.camName,minID)                               
#                print("PV ",length)
                LimelightHelpers.set_botpose_estimate_wpiblue_megatag2(
                    estimatedRobotPose.estimatedPose.toPose2d(),
                    result.getTimestampSeconds(),
                    result.getLatencyMillis(), #latency
                    length,  #number of tags
                    avg_dist,  # tag span (the distance betweebn the two furthestr tags) 
                    avg_dist, #avg_tag_dist, 
                    avg_area, # avg_tag_area, 
                    rawFid,   # list of LL raw fiducials
                    True, # is_megatag_2
                    self.camName
                )

            
            
