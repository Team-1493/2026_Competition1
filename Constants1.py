from math import pi
from robotpy_apriltag import  AprilTag,AprilTagField,AprilTagFieldLayout
from wpilib import SmartDashboard
from wpimath.geometry import Pose3d, Rotation3d,Translation3d

class ConstantValues():
    instance=None
    
    @staticmethod
    def getInstance():
        if ConstantValues.instance == None:
            ConstantValues.instance = ConstantValues()
            print("**********************  ConstantValues  **********************") 
            ConstantValues.write_constants_dashboard()
        return ConstantValues.instance
    
    class DriveConstants():
        
        # modify these in tuner_constants also!
        TELEOP_kP = 5.5 #TunerConstants._drive_gains.k_p
        TELEOP_kS = 2 #TunerConstants._drive_gains.k_s
        TELEOP_kV = 0 # TunerConstants._drive_gains.k_v
        TELEOP_kA = 0 # TunerConstants._drive_gains.k_a
        TELEOP_DEADBAND = 0.0025
        TELEOP_MAX_ANGULAR_RATE = 1
        TELEOP_SCALE_FACTOR_XY = 0.3
        TELEOP_SCALE_FACTOR_ROT = 0.3
        SPEED_AT_12_VOLTS = 3.6#TunerConstants.speed_at_12_volts 


        AUTO_kP = 3.45
        AUTO_kS = 0
        AUTO_kV = 0
        AUTO_kA = 0    

        

    class LimelightConstants():
        CAM_NAME=[None,None,None,None]
        CAM_X_OFFSET=[None,None,None,None]
        CAM_Y_OFFSET=[None,None,None,None]
        CAM_Z_OFFSET=[None,None,None,None]
        CAM_THETA_X_OFFSET=[None,None,None,None]
        CAM_THETA_Y_OFFSET=[None,None,None,None]
        CAM_THETA_Z_OFFSET=[None,None,None,None]

        # limelight 4 on practice robot
        CAM_NAME[0] =  "limelight-a"  
        CAM_X_OFFSET[0] = -.135 # forward positive
        CAM_Y_OFFSET[0] = .077 # right positive
        CAM_Z_OFFSET[0] = .552 # up positive
        CAM_THETA_X_OFFSET[0] = 0 # roll
        CAM_THETA_Y_OFFSET[0] = 0 # pitch
        CAM_THETA_Z_OFFSET[0] = 0 # yaw

        CAM_NAME[1] =  "limelight-c"
        CAM_X_OFFSET[1] = 0 # forward positive
        CAM_Y_OFFSET[1] = 0 # right positive
        CAM_Z_OFFSET[1] = .85 # up positive
        CAM_THETA_X_OFFSET[1] = 0 # roll
        CAM_THETA_Y_OFFSET[1] = 0 # pitch
        CAM_THETA_Z_OFFSET[1] = 0 # yaw

        CAM_NAME[2] =  "limelight-a"
        CAM_X_OFFSET[2] = 0 # forward positive
        CAM_Y_OFFSET[2] = 0 # right positive
        CAM_Z_OFFSET[2] = 0 # up positive
        CAM_THETA_X_OFFSET[2] = 0 # roll
        CAM_THETA_Y_OFFSET[2] = 0 # pitch
        CAM_THETA_Z_OFFSET[2] = 0 # yaw

        STD_DEV_COEFF_XY = .1 #0.05
        STD_DEV_COEFF_THETA = 999 #0.04 or self,max_value

        CAMERA_CUTOFF_DISTANCE = 3 # meters, above this distance std's set to max        


    class AutoBuilderConstants():
        AUTOBUILDER_XY_kP = 8        
        AUTOBUILDER_XY_kD = 0
        AUTOBUILDER_THETA_kP = 5        
        AUTOBUILDER_THETA_kD = 0        

    class HeadingControllerConstants():
        
        HEADINGCONTROLLER_KP = 2.5
        HEADINGCONTROLLER_KD = 0.0
        HEADINGCONTROLLER_VMAX = 2.0
        HEADINGCONTROLLER_AMAX = 3.0 
        HEADINGCONTROLLER_TOLERANCE = 0.017
        HEADINGCONTROLLER_RATE_TOLERANCE = 0.03                 
    
    class DriveToGoalCameraConstants():
        kLateralTolerance = .5
        kForwardTolerance = 0.03 # meters
        kRotTolerance = 0.017
        kEndTriggerDebounce = 0.06 #seconds
        kTimeoutTeleop = 1.5 #seconds
        kTimeoutAuto = 0.6 #seconds
        kForVmax = 1
        kForAmax = 6
        kForP = 8                
        kForD = .1
        kLatVmax = .5
        kLatAmax = 1
        kLatP = 0.2               
        kLatD = 0.0
        kRotVmax = 2
        kRotAmax = 4
        kRotP = 12                
        kRotD = 1    

        
    class VisionConstants():    
        
        tag1 = AprilTag()
        tag2 = AprilTag()
        tag3 = AprilTag()
        tag4 = AprilTag()
        tag5 = AprilTag()
        tag6 = AprilTag()
        tag7 = AprilTag()
        tag8 = AprilTag()
        tag9 = AprilTag()
        tag10 = AprilTag()
        tag11 = AprilTag()
        tag12 = AprilTag()
        tag13 = AprilTag()
        tag14 = AprilTag()
        tag15 = AprilTag()
        tag16 = AprilTag()
        tag17 = AprilTag()
        tag18 = AprilTag()
        tag19 = AprilTag()
        tag20 = AprilTag()
        tag21 = AprilTag()
        tag22 = AprilTag()
        tag23 = AprilTag()
        tag24 = AprilTag()
        tag25 = AprilTag()
        tag26 = AprilTag()
        tag27 = AprilTag()
        tag28 = AprilTag()
        tag29 = AprilTag()
        tag30 = AprilTag()
        tag31 = AprilTag()
        tag32 = AprilTag()

        tag1.ID = 1
        tag2.ID = 2
        tag3.ID = 3
        tag4.ID = 4
        tag5.ID = 5
        tag6.ID = 6
        tag7.ID = 7
        tag8.ID = 8
        tag9.ID = 9
        tag10.ID = 10
        tag11.ID = 11
        tag12.ID = 12
        tag13.ID = 13
        tag14.ID = 14
        tag15.ID = 15
        tag16.ID = 16
        tag17.ID = 17
        tag18.ID = 18
        tag19.ID = 19
        tag20.ID = 20
        tag21.ID = 21
        tag22.ID = 22
        tag23.ID = 23
        tag24.ID = 24
        tag25.ID = 25
        tag26.ID = 26
        tag27.ID = 27
        tag28.ID = 28
        tag29.ID = 29
        tag30.ID = 30
        tag31.ID = 31
        tag32.ID = 32

        tag1.pose = Pose3d(Translation3d(11.878, 7.425, 0.889), Rotation3d(0.000, 0.000, 3.142))
        tag2.pose = Pose3d(Translation3d(11.915, 4.638, 1.124), Rotation3d(0.000, 0.000, 1.571))
        tag3.pose = Pose3d(Translation3d(11.312, 4.390, 1.124), Rotation3d(0.000, 0.000, 3.142))
        tag4.pose = Pose3d(Translation3d(11.312, 4.035, 1.124), Rotation3d(0.000, 0.000, 3.142))
        tag5.pose = Pose3d(Translation3d(11.915, 3.431, 1.124), Rotation3d(0.000, 0.000, 4.712))
        tag6.pose = Pose3d(Translation3d(11.878, 0.644, 0.889), Rotation3d(0.000, 0.000, 3.142))
        tag7.pose = Pose3d(Translation3d(11.953, 0.644, 0.889), Rotation3d(0.000, 0.000, 0.000))
        tag8.pose = Pose3d(Translation3d(12.271, 3.431, 1.124), Rotation3d(0.000, 0.000, 4.712))
        tag9.pose = Pose3d(Translation3d(12.519, 3.679, 1.124), Rotation3d(0.000, 0.000, 0.000))
        tag10.pose = Pose3d(Translation3d(12.519, 4.035, 1.124), Rotation3d(0.000, 0.000, 0.000))
        tag11.pose = Pose3d(Translation3d(12.271, 4.638, 1.124), Rotation3d(0.000, 0.000, 1.571))
        tag12.pose = Pose3d(Translation3d(11.953, 7.425, 0.889), Rotation3d(0.000, 0.000, 0.000))
        tag13.pose = Pose3d(Translation3d(16.533, 7.403, 0.552), Rotation3d(0.000, 0.000, 3.142))
        tag14.pose = Pose3d(Translation3d(16.533, 6.972, 0.552), Rotation3d(0.000, 0.000, 3.142))
        tag15.pose = Pose3d(Translation3d(16.533, 4.324, 0.552), Rotation3d(0.000, 0.000, 3.142))
        tag16.pose = Pose3d(Translation3d(16.533, 3.892, 0.552), Rotation3d(0.000, 0.000, 3.142))
        tag17.pose = Pose3d(Translation3d(4.663, 0.644, 0.889), Rotation3d(0.000, 0.000, 0.000))
        tag18.pose = Pose3d(Translation3d(4.626, 3.431, 1.124), Rotation3d(0.000, 0.000, 4.712))
        tag19.pose = Pose3d(Translation3d(5.229, 3.679, 1.124), Rotation3d(0.000, 0.000, 0.000))
        tag20.pose = Pose3d(Translation3d(5.229, 4.035, 1.124), Rotation3d(0.000, 0.000, 0.000))
        tag21.pose = Pose3d(Translation3d(4.626, 4.638, 1.124), Rotation3d(0.000, 0.000, 1.571))
        tag22.pose = Pose3d(Translation3d(4.663, 7.425, 0.889), Rotation3d(0.000, 0.000, 0.000))
        tag23.pose = Pose3d(Translation3d(4.588, 7.425, 0.889), Rotation3d(0.000, 0.000, 3.142))
        tag24.pose = Pose3d(Translation3d(4.270, 4.638, 1.124), Rotation3d(0.000, 0.000, 1.571))
        tag25.pose = Pose3d(Translation3d(4.022, 4.390, 1.124), Rotation3d(0.000, 0.000, 3.142))
        tag26.pose = Pose3d(Translation3d(4.022, 4.035, 1.124), Rotation3d(0.000, 0.000, 3.142))
        tag27.pose = Pose3d(Translation3d(4.270, 3.431, 1.124), Rotation3d(0.000, 0.000, 4.712))
        tag28.pose = Pose3d(Translation3d(4.588, 0.644, 0.889), Rotation3d(0.000, 0.000, 3.142))
        tag29.pose = Pose3d(Translation3d(0.008, 0.666, 0.552), Rotation3d(0.000, 0.000, 0.000))
        tag30.pose = Pose3d(Translation3d(0.008, 1.098, 0.552), Rotation3d(0.000, 0.000, 0.000))
        tag31.pose = Pose3d(Translation3d(0.008, 3.746, 0.552), Rotation3d(0.000, 0.000, 0.000))
        tag32.pose = Pose3d(Translation3d(0.008, 4.178, 0.552), Rotation3d(0.000, 0.000, 0.000))



        # tags facing robot have r3 = 180 degrees
        
        tags_list = [tag1,tag2,tag3,tag4,tag5,tag6,tag7,tag8,tag9,tag10,tag11,
                    tag12,tag13,tag14,tag15,tag16,tag17,tag18,tag19,tag20,tag21,
                    tag22, tag23,tag24, tag25,tag26,tag27,tag28,tag29,tag30,tag31,tag32]
        
        
        field_length_meters = 16.533 # Example field dimensions
        field_width_meters = 8.11 # Example field dimensions
        field_layout = AprilTagFieldLayout(tags_list, field_length_meters, field_width_meters)
        
        

    #    field_layout = AprilTagFieldLayout("deploy/apriltags/2026_field.json")
    #    tags_list=field_layout.getTags()   
    
    @staticmethod
    def update_constants():

       ### Update Constants for Drive
        ConstantValues.DriveConstants.TELEOP_kP = SmartDashboard.getNumber("Drive Teleop kP",ConstantValues.DriveConstants.TELEOP_kP)
        ConstantValues.DriveConstants.TELEOP_kS = SmartDashboard.getNumber("Drive Teleop kS",ConstantValues.DriveConstants.TELEOP_kS) 
        ConstantValues.DriveConstants.TELEOP_kV = SmartDashboard.getNumber("Drive Teleop kV",ConstantValues.DriveConstants.TELEOP_kV)
        ConstantValues.DriveConstants.TELEOP_kA = SmartDashboard.getNumber("Drive Teleop kA",ConstantValues.DriveConstants.TELEOP_kA)                           
        ConstantValues.DriveConstants.AUTO_kP = SmartDashboard.getNumber("Drive Auto kP",ConstantValues.DriveConstants.AUTO_kP)
        ConstantValues.DriveConstants.AUTO_kS = SmartDashboard.getNumber("Drive Auto kS",ConstantValues.DriveConstants.AUTO_kS)
        ConstantValues.DriveConstants.AUTO_kV = SmartDashboard.getNumber("Drive Auto kV",ConstantValues.DriveConstants.AUTO_kV)
        ConstantValues.DriveConstants.AUTO_kA = SmartDashboard.getNumber("Drive Auto kA",ConstantValues.DriveConstants.AUTO_kA)                         

        ConstantValues.DriveConstants.TELEOP_DEADBAND = SmartDashboard.getNumber("Drive Teleop Deadband",ConstantValues.DriveConstants.TELEOP_DEADBAND)                         
        ConstantValues.DriveConstants.TELEOP_MAX_ANGULAR_RATE = SmartDashboard.getNumber("Drive Teleop MaxAngRate",ConstantValues.DriveConstants.TELEOP_MAX_ANGULAR_RATE)
        ConstantValues.DriveConstants.TELEOP_SCALE_FACTOR_XY = SmartDashboard.getNumber("Drive Teleop Scale XY",ConstantValues.DriveConstants.TELEOP_SCALE_FACTOR_XY)
        ConstantValues.DriveConstants.TELEOP_SCALE_FACTOR_ROT = SmartDashboard.getNumber("Drive Teleop Scale Rot",ConstantValues.DriveConstants.TELEOP_SCALE_FACTOR_ROT)                                 
        ConstantValues.DriveConstants.SPEED_AT_12_VOLTS = SmartDashboard.getNumber("Drive Speed 12V",ConstantValues.DriveConstants.SPEED_AT_12_VOLTS)                         


  
        ### Update Constants for DriveToGoalCamera
        ConstantValues.DriveToGoalCameraConstants.kLateralTolerance = SmartDashboard.getNumber("DTG_Cam LatTol",ConstantValues.DriveToGoalCameraConstants.kLateralTolerance)      
        ConstantValues.DriveToGoalCameraConstants.kForwardTolerance = SmartDashboard.getNumber("DTG_Cam ForTol",ConstantValues.DriveToGoalCameraConstants.kForwardTolerance)
        ConstantValues.DriveToGoalCameraConstants.kRotTolerance = SmartDashboard.getNumber("DTG_Cam RotTol",ConstantValues.DriveToGoalCameraConstants.kRotTolerance)
        ConstantValues.DriveToGoalCameraConstants.kEndTriggerDebounce = SmartDashboard.getNumber("DTG_Cam Debounce",ConstantValues.DriveToGoalCameraConstants.kEndTriggerDebounce)
        ConstantValues.DriveToGoalCameraConstants.kTimeoutTeleop = SmartDashboard.getNumber("DTG_Cam Timeout Tele",ConstantValues.DriveToGoalCameraConstants.kTimeoutTeleop)
        ConstantValues.DriveToGoalCameraConstants.kTimeoutAuto = SmartDashboard.getNumber("DTG_Cam TimeoutAuto",ConstantValues.DriveToGoalCameraConstants.kTimeoutAuto)    
        ConstantValues.DriveToGoalCameraConstants.kForP = SmartDashboard.getNumber("DTG_Cam For kP",ConstantValues.DriveToGoalCameraConstants.kForP)
        ConstantValues.DriveToGoalCameraConstants.kForD = SmartDashboard.getNumber("DTG_Cam For kD",ConstantValues.DriveToGoalCameraConstants.kForD)
        ConstantValues.DriveToGoalCameraConstants.kForVmax = SmartDashboard.getNumber("DTG_Cam For Vmax",ConstantValues.DriveToGoalCameraConstants.kForVmax) 
        ConstantValues.DriveToGoalCameraConstants.kForAmax = SmartDashboard.getNumber("DTG_Cam For Amax",ConstantValues.DriveToGoalCameraConstants.kForAmax)                 
        ConstantValues.DriveToGoalCameraConstants.kLatP = SmartDashboard.getNumber("DTG_Cam Lat kP",ConstantValues.DriveToGoalCameraConstants.kLatP)
        ConstantValues.DriveToGoalCameraConstants.kLatD = SmartDashboard.getNumber("DTG_Cam Lat kD",ConstantValues.DriveToGoalCameraConstants.kLatD)
        ConstantValues.DriveToGoalCameraConstants.kLatVmax = SmartDashboard.getNumber("DTG_Cam Lat Vmax",ConstantValues.DriveToGoalCameraConstants.kLatVmax) 
        ConstantValues.DriveToGoalCameraConstants.kLatAmax = SmartDashboard.getNumber("DTG_Cam Lat Amax",ConstantValues.DriveToGoalCameraConstants.kLatAmax)                 
        ConstantValues.DriveToGoalCameraConstants.kRotP = SmartDashboard.getNumber("DTG_Cam Rot kP",ConstantValues.DriveToGoalCameraConstants.kRotP)
        ConstantValues.DriveToGoalCameraConstants.kRotD = SmartDashboard.getNumber("DTG_Cam Rot kD",ConstantValues.DriveToGoalCameraConstants.kRotD)
        ConstantValues.DriveToGoalCameraConstants.kRotVmax = SmartDashboard.getNumber("DTG_Cam Rot Vmax",ConstantValues.DriveToGoalCameraConstants.kRotVmax) 
        ConstantValues.DriveToGoalCameraConstants.kRotAmax = SmartDashboard.getNumber("DTG_Cam Rot Amax",ConstantValues.DriveToGoalCameraConstants.kRotAmax)   


        ## Update values for limelimesystyem
        ConstantValues.LimelightConstants.CAM_NAME[0] = SmartDashboard.getString("LL Cam0 Name", ConstantValues.LimelightConstants.CAM_NAME[0])
        ConstantValues.LimelightConstants.CAM_X_OFFSET[0] = SmartDashboard.getNumber("LL CAM0 x_offset",ConstantValues.LimelightConstants.CAM_X_OFFSET[0]) 
        ConstantValues.LimelightConstants.CAM_Y_OFFSET[0] = SmartDashboard.getNumber("LL CAM0 y_offset",ConstantValues.LimelightConstants.CAM_Y_OFFSET[0])
        ConstantValues.LimelightConstants.CAM_Z_OFFSET[0] = SmartDashboard.getNumber("LL CAM0 z_offset",ConstantValues.LimelightConstants.CAM_Z_OFFSET[0]) 
        ConstantValues.LimelightConstants.CAM_THETA_X_OFFSET[0] = SmartDashboard.getNumber("LL CAM0 thetaX_offset",ConstantValues.LimelightConstants.CAM_THETA_X_OFFSET[0])
        ConstantValues.LimelightConstants.CAM_THETA_Y_OFFSET[0] = SmartDashboard.getNumber("LL CAM0 thetaY_offset",ConstantValues.LimelightConstants.CAM_THETA_Y_OFFSET[0])
        ConstantValues.LimelightConstants.CAM_THETA_Z_OFFSET[0] = SmartDashboard.getNumber("LL CAM0 thetaZ_offset",ConstantValues.LimelightConstants.CAM_THETA_Z_OFFSET[0])                 
        ConstantValues.LimelightConstants.CAM_NAME[1] = SmartDashboard.getString("LL CAM1 Name", ConstantValues.LimelightConstants.CAM_NAME[1])
        ConstantValues.LimelightConstants.CAM_X_OFFSET[1] = SmartDashboard.getNumber("LL CAM1 x_offset",ConstantValues.LimelightConstants.CAM_X_OFFSET[1]) 
        ConstantValues.LimelightConstants.CAM_Y_OFFSET[1] = SmartDashboard.getNumber("LL CAM1 y_offset",ConstantValues.LimelightConstants.CAM_Y_OFFSET[1])
        ConstantValues.LimelightConstants.CAM_Z_OFFSET[1] = SmartDashboard.getNumber("LL CAM1 z_offset",ConstantValues.LimelightConstants.CAM_Z_OFFSET[1]) 
        ConstantValues.LimelightConstants.CAM_THETA_X_OFFSET[1] = SmartDashboard.getNumber("LL CAM1 thetaX_offset",ConstantValues.LimelightConstants.CAM_THETA_X_OFFSET[1])
        ConstantValues.LimelightConstants.CAM_THETA_Y_OFFSET[1] = SmartDashboard.getNumber("LL CAM1 thetaY_offset",ConstantValues.LimelightConstants.CAM_THETA_Y_OFFSET[1])
        ConstantValues.LimelightConstants.CAM_THETA_Z_OFFSET[1] = SmartDashboard.getNumber("LL CAM1 thetaZ_offset",ConstantValues.LimelightConstants.CAM_THETA_Z_OFFSET[1])                 
        ConstantValues.LimelightConstants.STD_DEV_COEFF_XY = SmartDashboard.getNumber("LL StdDevCoeff_xy",ConstantValues.LimelightConstants.STD_DEV_COEFF_XY)
        ConstantValues.LimelightConstants.STD_DEV_COEFF_THETA = SmartDashboard.getNumber("LL StdDevCoeff_theta",ConstantValues.LimelightConstants.STD_DEV_COEFF_THETA)
        ConstantValues.LimelightConstants.CAMERA_CUTOFF_DISTANCE = SmartDashboard.getNumber("LL CutoffDist",ConstantValues.LimelightConstants.CAMERA_CUTOFF_DISTANCE)
        
        # Update values for autobuilder
        ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kP=  SmartDashboard.getNumber("AutoBuilder XY_kP",ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kP)
        ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kD=  SmartDashboard.getNumber("AutoBuilder XY_kD",ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kD)        
        ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kP=  SmartDashboard.getNumber("AutoBuilder THETA_kP",ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kP)
        ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kD=  SmartDashboard.getNumber("AutoBuilder THETA_kD",ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kD)

        # update values for heading controller
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KP =  SmartDashboard.getNumber("HeadingController kP",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KP)
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KD =  SmartDashboard.getNumber("HeadingController kD",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KD)
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_VMAX =  SmartDashboard.getNumber("HeadingController Vmax",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_VMAX) 
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_AMAX =  SmartDashboard.getNumber("HeadingController Amax",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_AMAX)                               
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_TOLERANCE =  SmartDashboard.getNumber("HeadingController Tol",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_TOLERANCE)
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_RATE_TOLERANCE =  SmartDashboard.getNumber("HeadingController Rate Tol",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_RATE_TOLERANCE)


 
    @staticmethod
    def write_constants_dashboard():     
        
        SmartDashboard.putNumber("Drive Teleop kP",ConstantValues.DriveConstants.TELEOP_kP)
        SmartDashboard.putNumber("Drive Teleop kS",ConstantValues.DriveConstants.TELEOP_kS) 
        SmartDashboard.putNumber("Drive Teleop kV",ConstantValues.DriveConstants.TELEOP_kV)
        SmartDashboard.putNumber("Drive Teleop kA",ConstantValues.DriveConstants.TELEOP_kA)                           
        SmartDashboard.putNumber("Drive Auto kP",ConstantValues.DriveConstants.AUTO_kP)
        SmartDashboard.putNumber("Drive Auto kS",ConstantValues.DriveConstants.AUTO_kS)
        SmartDashboard.putNumber("Drive Auto kV",ConstantValues.DriveConstants.AUTO_kV)
        SmartDashboard.putNumber("Drive Auto kA",ConstantValues.DriveConstants.AUTO_kA)    

        SmartDashboard.putNumber("Drive Teleop Deadband",ConstantValues.DriveConstants.TELEOP_DEADBAND)                         
        SmartDashboard.putNumber("Drive Teleop MaxAngRate",ConstantValues.DriveConstants.TELEOP_MAX_ANGULAR_RATE)
        SmartDashboard.putNumber("Drive Teleop Scale XY",ConstantValues.DriveConstants.TELEOP_SCALE_FACTOR_XY)
        SmartDashboard.putNumber("Drive Teleop Scale Rot",ConstantValues.DriveConstants.TELEOP_SCALE_FACTOR_ROT)                                 
        SmartDashboard.putNumber("Drive Speed 12V",ConstantValues.DriveConstants.SPEED_AT_12_VOLTS)                         
                    

        SmartDashboard.putNumber("DTG_Cam LatTol",ConstantValues.DriveToGoalCameraConstants.kLateralTolerance)      
        SmartDashboard.putNumber("DTG_Cam ForTol",ConstantValues.DriveToGoalCameraConstants.kForwardTolerance)
        SmartDashboard.putNumber("DTG_Cam RotTol",ConstantValues.DriveToGoalCameraConstants.kRotTolerance)
        SmartDashboard.putNumber("DTG_Cam Debounce",ConstantValues.DriveToGoalCameraConstants.kEndTriggerDebounce)
        SmartDashboard.putNumber("DTG_Cam Timeout Tele",ConstantValues.DriveToGoalCameraConstants.kTimeoutTeleop)
        SmartDashboard.putNumber("DTG_Cam TimeoutAuto",ConstantValues.DriveToGoalCameraConstants.kTimeoutAuto)    
        SmartDashboard.putNumber("DTG_Cam For kP",ConstantValues.DriveToGoalCameraConstants.kForP)
        SmartDashboard.putNumber("DTG_Cam For kD",ConstantValues.DriveToGoalCameraConstants.kForD)
        SmartDashboard.putNumber("DTG_Cam For Vmax",ConstantValues.DriveToGoalCameraConstants.kForVmax) 
        SmartDashboard.putNumber("DTG_Cam For Amax",ConstantValues.DriveToGoalCameraConstants.kForAmax)                 
        SmartDashboard.putNumber("DTG_Cam Lat kP",ConstantValues.DriveToGoalCameraConstants.kLatP)
        SmartDashboard.putNumber("DTG_Cam Lat kD",ConstantValues.DriveToGoalCameraConstants.kLatD)
        SmartDashboard.putNumber("DTG_Cam Lat Vmax",ConstantValues.DriveToGoalCameraConstants.kLatVmax) 
        SmartDashboard.putNumber("DTG_Cam Lat Amax",ConstantValues.DriveToGoalCameraConstants.kLatAmax)                 
        SmartDashboard.putNumber("DTG_Cam Rot kP",ConstantValues.DriveToGoalCameraConstants.kRotP)
        SmartDashboard.putNumber("DTG_Cam Rot kD",ConstantValues.DriveToGoalCameraConstants.kRotD)
        SmartDashboard.putNumber("DTG_Cam Rot Vmax",ConstantValues.DriveToGoalCameraConstants.kRotVmax) 
        SmartDashboard.putNumber("DTG_Cam Rot Amax",ConstantValues.DriveToGoalCameraConstants.kRotAmax) 


        SmartDashboard.putString("LL CAM0 Cam Name", ConstantValues.LimelightConstants.CAM_NAME[0])
        SmartDashboard.putNumber("LL CAM0 x_offset",ConstantValues.LimelightConstants.CAM_X_OFFSET[0]) 
        SmartDashboard.putNumber("LL CAM0 y_offset",ConstantValues.LimelightConstants.CAM_Y_OFFSET[0])
        SmartDashboard.putNumber("LL CAM0 z_offset",ConstantValues.LimelightConstants.CAM_Z_OFFSET[0]) 
        SmartDashboard.putNumber("LL CAM0 thetaX_offset",ConstantValues.LimelightConstants.CAM_THETA_X_OFFSET[0])
        SmartDashboard.putNumber("LL CAM0 thetaY_offset",ConstantValues.LimelightConstants.CAM_THETA_Y_OFFSET[0])
        SmartDashboard.putNumber("LL CAM0 thetaZ_offset",ConstantValues.LimelightConstants.CAM_THETA_Z_OFFSET[0])                 
        SmartDashboard.putString("LL CAM1 Cam Name", ConstantValues.LimelightConstants.CAM_NAME[1])
        SmartDashboard.putNumber("LL CAM1 x_offset",ConstantValues.LimelightConstants.CAM_X_OFFSET[1]) 
        SmartDashboard.putNumber("LL CAM1 y_offset",ConstantValues.LimelightConstants.CAM_Y_OFFSET[1])
        SmartDashboard.putNumber("LL CAM1 z_offset",ConstantValues.LimelightConstants.CAM_Z_OFFSET[1]) 
        SmartDashboard.putNumber("LL CAM1 thetaX_offset",ConstantValues.LimelightConstants.CAM_THETA_X_OFFSET[1])
        SmartDashboard.putNumber("LL CAM1 thetaY_offset",ConstantValues.LimelightConstants.CAM_THETA_Y_OFFSET[1])
        SmartDashboard.putNumber("LL CAM1 thetaZ_offset",ConstantValues.LimelightConstants.CAM_THETA_Z_OFFSET[1])                 
        SmartDashboard.putNumber("LL StdDevCoeff_xy",ConstantValues.LimelightConstants.STD_DEV_COEFF_XY)
        SmartDashboard.putNumber("LL StdDevCoeff_theta",ConstantValues.LimelightConstants.STD_DEV_COEFF_THETA)
        SmartDashboard.putNumber("LL CutoffDist",ConstantValues.LimelightConstants.CAMERA_CUTOFF_DISTANCE)


        SmartDashboard.putNumber("AutoBuilder XY_kP",ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kP)
        SmartDashboard.putNumber("AutoBuilder XY_kD",ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kD)
        SmartDashboard.putNumber("AutoBuilder THETA_kP",ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kP)
        SmartDashboard.putNumber("AutoBuilder THETA_kD",ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kD)


        SmartDashboard.putNumber("HeadingController kP",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KP)
        SmartDashboard.putNumber("HeadingController kD",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KD)
        SmartDashboard.putNumber("HeadingController Vmax",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_VMAX) 
        SmartDashboard.putNumber("HeadingController Amax",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_AMAX)                               
        SmartDashboard.putNumber("HeadingController Tol",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_TOLERANCE)
        SmartDashboard.putNumber("HeadingController Rate Tol",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_RATE_TOLERANCE)


