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
        TELEOP_kP =  3.45 #5.5 TunerConstants._drive_gains.k_p
        TELEOP_kS = 0 #TunerConstants._drive_gains.k_s
        TELEOP_kV = 0 # TunerConstants._drive_gains.k_v
        TELEOP_kA = 0 # TunerConstants._drive_gains.k_a
        TELEOP_DEADBAND = 0.0025
        TELEOP_MAX_ANGULAR_RATE = 1
        TELEOP_SCALE_FACTOR_XY = 0.8
        TELEOP_SCALE_FACTOR_ROT = 0.8
        SPEED_AT_12_VOLTS = 3.3#TunerConstants.speed_at_12_volts 


        AUTO_kP = 5.5  #3.45  
        AUTO_kS = .5
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
        CAM_NAME[0] =  "limelight-b"  
        CAM_X_OFFSET[0] = -0.11 #-.127 # forward positive
        CAM_Y_OFFSET[0] = 0.0 # left positive
        CAM_Z_OFFSET[0] = 0.7652 #.7 # up positive
        CAM_THETA_X_OFFSET[0] = 0 # roll
        CAM_THETA_Y_OFFSET[0] = 23 # pitch
        CAM_THETA_Z_OFFSET[0] = 0 # yaw

        CAM_NAME[1] =  "limelight-a"
        CAM_X_OFFSET[1] = 0.07 #-0.292 # forward positive
        CAM_Y_OFFSET[1] = .14 #-0.3 # left positive
        CAM_Z_OFFSET[1] = 0.7652 #0.8 # up positive
        CAM_THETA_X_OFFSET[1] = 0 # roll
        CAM_THETA_Y_OFFSET[1] = 23 # pitch
        CAM_THETA_Z_OFFSET[1] = 180 # yaw


        STD_DEV_COEFF_XY = .1 #0.05
        STD_DEV_COEFF_THETA = 999 #0.04 or self,max_value

        CAMERA_CUTOFF_DISTANCE_1 = 4 # meters, above this distance std's set to max
        CAMERA_CUTOFF_DISTANCE_2 = 4 # meters, above this distance std's set to max                
        CAMERA_CUTOFF_DIFFERENCE = 8 # meters, above this distance std's set to max                


    class AutoBuilderConstants():
        AUTOBUILDER_XY_kP = 8        
        AUTOBUILDER_XY_kD = 0
        AUTOBUILDER_THETA_kP = 5        
        AUTOBUILDER_THETA_kD = 0        

    class HeadingControllerConstants():
        
        HEADINGCONTROLLER_KP = 4#2.5
        HEADINGCONTROLLER_KD = 0.0
        HEADINGCONTROLLER_VMAX = 5#2.0
        HEADINGCONTROLLER_TOLERANCE = 0.017
        HEADINGCONTROLLER_RATE_TOLERANCE = 0.03
        SNAP_AWAY_FROM_WALL_DISTANCE = 1.1  # meters from nearest field wall
        SNAP_WALL_SWING_CLEARANCE = 0.9  # meters where front swing toward nearest wall is considered risky
        SNAP_FORCED_KP = 4  # rad/s commanded per rad of unwrapped heading error in forced-snap mode
        SNAP_FORCED_MIN_RATE = 0.35  # rad/s minimum command while outside     
    

    class IntakeConstants():
        INTAKE_VOLTAGE = -8
        INTAKE_AUTO_VOLTAGE = -8
        INTAKE_MAX_VEL =10    
        INTAKE_STATOR_CL = 25
        INTAKE_SUPPLY_CL = 25
        INTAKE_LOWERLIMIT_CL = 20
        INTAKE_LOWERTIME_CL = 0.25
        INTAKE_CONVEYOR_VOLTAGE = -10
        INTAKE_CONVEYOR_SLOW_VOLTAGE = -4        
        MAX_UP_ROTATION = .26
        MAX_DOWN_ROTATION = 0.00
        ARM_KP = 50
        ARM_KD = 0
        ARM_KI = 5
        ARM_KG = .35
        ARM_PEAK_FORWARD_TORQUE_CURRENT = 40
        ARM_PEAK_REVERSE_TORQUE_CURRENT = 40
        ARM_FORWARDTHRESH = .285
        ARM_REVERSETHRESH = 0                
        SENSOR_TO_MECHANISM_RATIO = 50 # 1 if 1 rotation of the motor = 1 rotation of the arm


    class ShooterConstants():
        LEADER_KP = 0.9
        LEADER_KS = 0 ##Caliberated by Sai & Mohammed
        LEADER_KV = 0.178 #Caliberated by Sai & Mohammed
        FEEDER_KP = 1
        FEEDER_KV = 0.124
        SHOOTING_VELOCITY = -1
        CONVEYOR_VELOCITY = 80
        TOLERANCE = 1e-3

    
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

        ConstantValues.IntakeConstants.INTAKE_VOLTAGE = SmartDashboard.getNumber("Intake Intake Volt",ConstantValues.IntakeConstants.INTAKE_VOLTAGE)                         
        ConstantValues.IntakeConstants.INTAKE_AUTO_VOLTAGE = SmartDashboard.getNumber("Intake Auto Volt",ConstantValues.IntakeConstants.INTAKE_AUTO_VOLTAGE)                         
        ConstantValues.IntakeConstants.INTAKE_MAX_VEL = SmartDashboard.getNumber("Intake Max Vel",ConstantValues.IntakeConstants.INTAKE_MAX_VEL)
        ConstantValues.IntakeConstants.INTAKE_STATOR_CL = SmartDashboard.getNumber("Intake Stator CL",ConstantValues.IntakeConstants.INTAKE_STATOR_CL)
        ConstantValues.IntakeConstants.INTAKE_SUPPLY_CL = SmartDashboard.getNumber("Intake Suply CL",ConstantValues.IntakeConstants.INTAKE_SUPPLY_CL)        
        ConstantValues.IntakeConstants.INTAKE_LOWERLIMIT_CL = SmartDashboard.getNumber("Intake LowerLimit CL",ConstantValues.IntakeConstants.INTAKE_LOWERLIMIT_CL)       
        ConstantValues.IntakeConstants.INTAKE_LOWERTIME_CL = SmartDashboard.getNumber("Intake LowerTime CL",ConstantValues.IntakeConstants.INTAKE_LOWERTIME_CL)       

        ConstantValues.IntakeConstants.MAX_UP_ROTATION = SmartDashboard.getNumber("Intake Arm Max Up",ConstantValues.IntakeConstants.MAX_UP_ROTATION)
        ConstantValues.IntakeConstants.MAX_DOWN_ROTATION = SmartDashboard.getNumber("Intake Arm Max Down",ConstantValues.IntakeConstants.MAX_DOWN_ROTATION)   
        ConstantValues.IntakeConstants.INTAKE_CONVEYOR_VOLTAGE = SmartDashboard.getNumber("Intake Conveyor Voltage", ConstantValues.IntakeConstants.INTAKE_CONVEYOR_VOLTAGE)
        ConstantValues.IntakeConstants.INTAKE_CONVEYOR_SLOW_VOLTAGE = SmartDashboard.getNumber("Intake Conveyor Slow Voltage", ConstantValues.IntakeConstants.INTAKE_CONVEYOR_SLOW_VOLTAGE)        
        ConstantValues.IntakeConstants.ARM_KP = SmartDashboard.getNumber("Intake Arm kP",ConstantValues.IntakeConstants.ARM_KP)    
        ConstantValues.IntakeConstants.ARM_KI = SmartDashboard.getNumber("Intake Arm kI",ConstantValues.IntakeConstants.ARM_KI)
        ConstantValues.IntakeConstants.ARM_KD = SmartDashboard.getNumber("Intake Arm kD",ConstantValues.IntakeConstants.ARM_KD)                                                                           
        ConstantValues.IntakeConstants.ARM_KG = SmartDashboard.getNumber("Intake Arm kG",ConstantValues.IntakeConstants.ARM_KG)                     
        ConstantValues.IntakeConstants.ARM_PEAK_FORWARD_TORQUE_CURRENT = SmartDashboard.getNumber("Intake Arm Peak Curr For",ConstantValues.IntakeConstants.ARM_PEAK_FORWARD_TORQUE_CURRENT)                     
        ConstantValues.IntakeConstants.ARM_PEAK_REVERSE_TORQUE_CURRENT = SmartDashboard.getNumber("Intake Arm Peak Curr Rev",ConstantValues.IntakeConstants.ARM_PEAK_REVERSE_TORQUE_CURRENT)                     
        ConstantValues.IntakeConstants.ARM_FORWARDTHRESH = SmartDashboard.getNumber("Intake Arm For Thresh",ConstantValues.IntakeConstants.ARM_FORWARDTHRESH)                    
        ConstantValues.IntakeConstants.ARM_REVERSETHRESH = SmartDashboard.getNumber("Intake Arm Rev Thresh",ConstantValues.IntakeConstants.ARM_REVERSETHRESH)                     
        ConstantValues.IntakeConstants.SENSOR_TO_MECHANISM_RATIO = SmartDashboard.getNumber("Intake Arm Mech Ratio",ConstantValues.IntakeConstants.SENSOR_TO_MECHANISM_RATIO)                     


        ### Update Constants for Shoot
        ConstantValues.ShooterConstants.LEADER_KP = SmartDashboard.getNumber("Leader KP", ConstantValues.ShooterConstants.LEADER_KP)
        ConstantValues.ShooterConstants.LEADER_KV = SmartDashboard.getNumber("Leader KV", ConstantValues.ShooterConstants.LEADER_KV)
        ConstantValues.ShooterConstants.LEADER_KS = SmartDashboard.getNumber("Leader KS", ConstantValues.ShooterConstants.LEADER_KS)        
        ConstantValues.ShooterConstants.CONVEYOR_VELOCITY = SmartDashboard.getNumber("Conveyor Velocity", ConstantValues.ShooterConstants.CONVEYOR_VELOCITY)
        ConstantValues.ShooterConstants.FEEDER_KV = SmartDashboard.getNumber("Feeder KV", ConstantValues.ShooterConstants.FEEDER_KV)
        ConstantValues.ShooterConstants.FEEDER_KP = SmartDashboard.getNumber("Feeder KP", ConstantValues.ShooterConstants.FEEDER_KP)
        ConstantValues.ShooterConstants.SHOOTING_VELOCITY = SmartDashboard.getNumber("Shooting Velocity", ConstantValues.ShooterConstants.SHOOTING_VELOCITY)
        

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
        ConstantValues.LimelightConstants.CAMERA_CUTOFF_DISTANCE_1 = SmartDashboard.getNumber("LL CutoffDist_1",ConstantValues.LimelightConstants.CAMERA_CUTOFF_DISTANCE_1)
        ConstantValues.LimelightConstants.CAMERA_CUTOFF_DISTANCE_2 = SmartDashboard.getNumber("LL CutoffDist_2",ConstantValues.LimelightConstants.CAMERA_CUTOFF_DISTANCE_2)
        ConstantValues.LimelightConstants.CAMERA_CUTOFF_DIFFERENCE = SmartDashboard.getNumber("LL CutoffDifference",ConstantValues.LimelightConstants.CAMERA_CUTOFF_DIFFERENCE)

        # Update values for autobuilder
        ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kP=  SmartDashboard.getNumber("AutoBuilder XY_kP",ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kP)
        ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kD=  SmartDashboard.getNumber("AutoBuilder XY_kD",ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kD)        
        ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kP=  SmartDashboard.getNumber("AutoBuilder THETA_kP",ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kP)
        ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kD=  SmartDashboard.getNumber("AutoBuilder THETA_kD",ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kD)

        # update values for heading controller
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KP =  SmartDashboard.getNumber("HeadingController kP",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KP)
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KD =  SmartDashboard.getNumber("HeadingController kD",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KD)
        ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_VMAX =  SmartDashboard.getNumber("HeadingController Vmax",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_VMAX) 
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

        SmartDashboard.putNumber("Intake Intake Volt",ConstantValues.IntakeConstants.INTAKE_VOLTAGE)
        SmartDashboard.putNumber("Intake Auto Volt",ConstantValues.IntakeConstants.INTAKE_AUTO_VOLTAGE)
        SmartDashboard.putNumber("Intake Max Vel",ConstantValues.IntakeConstants.INTAKE_MAX_VEL)                                         
        SmartDashboard.putNumber("Intake Stator CL",ConstantValues.IntakeConstants.INTAKE_STATOR_CL)
        SmartDashboard.putNumber("Intake Suply CL",ConstantValues.IntakeConstants.INTAKE_SUPPLY_CL)        
        SmartDashboard.putNumber("Intake LowerLimit CL",ConstantValues.IntakeConstants.INTAKE_LOWERLIMIT_CL)       
        SmartDashboard.putNumber("Intake LowerTime CL",ConstantValues.IntakeConstants.INTAKE_LOWERTIME_CL)       
        SmartDashboard.putNumber("Intake Conveyor Voltage", ConstantValues.IntakeConstants.INTAKE_CONVEYOR_VOLTAGE)
        SmartDashboard.putNumber("Intake Conveyor Slow Voltage", ConstantValues.IntakeConstants.INTAKE_CONVEYOR_SLOW_VOLTAGE)        
        SmartDashboard.putNumber("Intake Arm Max Up",ConstantValues.IntakeConstants.MAX_UP_ROTATION)
        SmartDashboard.putNumber("Intake Arm Max Down",ConstantValues.IntakeConstants.MAX_DOWN_ROTATION)   
        SmartDashboard.putNumber("Intake Arm kP",ConstantValues.IntakeConstants.ARM_KP)    
        SmartDashboard.putNumber("Intake Arm kI",ConstantValues.IntakeConstants.ARM_KI)
        SmartDashboard.putNumber("Intake Arm kD",ConstantValues.IntakeConstants.ARM_KD)                                                                           
        SmartDashboard.putNumber("Intake Arm kG",ConstantValues.IntakeConstants.ARM_KG)                     
        SmartDashboard.putNumber("Intake Arm Peak Curr For",ConstantValues.IntakeConstants.ARM_PEAK_FORWARD_TORQUE_CURRENT)                     
        SmartDashboard.putNumber("Intake Arm Peak Curr Rev",ConstantValues.IntakeConstants.ARM_PEAK_REVERSE_TORQUE_CURRENT)                     
        SmartDashboard.putNumber("Intake Arm For Thresh",ConstantValues.IntakeConstants.ARM_FORWARDTHRESH)                     
        SmartDashboard.putNumber("Intake Arm Rev Thresh",ConstantValues.IntakeConstants.ARM_REVERSETHRESH)                     
        SmartDashboard.putNumber("Intake Arm Mech Ratio",ConstantValues.IntakeConstants.SENSOR_TO_MECHANISM_RATIO)                     

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
        SmartDashboard.putNumber("LL CutoffDist_1",ConstantValues.LimelightConstants.CAMERA_CUTOFF_DISTANCE_1)
        SmartDashboard.putNumber("LL CutoffDist_2",ConstantValues.LimelightConstants.CAMERA_CUTOFF_DISTANCE_2)
        SmartDashboard.putNumber("LL CutoffDiff",ConstantValues.LimelightConstants.CAMERA_CUTOFF_DIFFERENCE)        


        SmartDashboard.putNumber("AutoBuilder XY_kP",ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kP)
        SmartDashboard.putNumber("AutoBuilder XY_kD",ConstantValues.AutoBuilderConstants.AUTOBUILDER_XY_kD)
        SmartDashboard.putNumber("AutoBuilder THETA_kP",ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kP)
        SmartDashboard.putNumber("AutoBuilder THETA_kD",ConstantValues.AutoBuilderConstants.AUTOBUILDER_THETA_kD)


        SmartDashboard.putNumber("HeadingController kP",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KP)
        SmartDashboard.putNumber("HeadingController kD",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KD)
        SmartDashboard.putNumber("HeadingController Vmax",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_VMAX) 
        SmartDashboard.putNumber("HeadingController Tol",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_TOLERANCE)
        SmartDashboard.putNumber("HeadingController Rate Tol",ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_RATE_TOLERANCE)

        SmartDashboard.putNumber("Leader KP", ConstantValues.ShooterConstants.LEADER_KP)
        SmartDashboard.putNumber("Leader KV", ConstantValues.ShooterConstants.LEADER_KV)
        SmartDashboard.putNumber("Leader KS", ConstantValues.ShooterConstants.LEADER_KS)        
        SmartDashboard.putNumber("Feeder KV", ConstantValues.ShooterConstants.FEEDER_KV)
        SmartDashboard.putNumber("Feeder KP", ConstantValues.ShooterConstants.FEEDER_KP)
        SmartDashboard.putNumber("Shooting Velocity", ConstantValues.ShooterConstants.SHOOTING_VELOCITY)
        SmartDashboard.putNumber("Conveyor Velocity", ConstantValues.ShooterConstants.CONVEYOR_VELOCITY)
