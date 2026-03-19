from commands2 import Subsystem
from wpilib import SmartDashboard, DigitalInput
import wpilib
from phoenix6 import hardware, configs, controls,signals
from phoenix6.signals import GravityTypeValue
from Constants1 import ConstantValues

class IntakeSystem(Subsystem):
    instance=None
    
    @staticmethod
    def getInstance():
        if IntakeSystem.instance == None:
            IntakeSystem.instance = IntakeSystem(10,9,8,9)
            print('*' * 22 + ' INTAKE ' + '*' * 22)
        return IntakeSystem.instance
    def setup(self):
        self.voltage = ConstantValues.IntakeConstants.INTAKE_VOLTAGE

        self.goal_down = ConstantValues.IntakeConstants.MAX_DOWN_ROTATION
        self.goal_up = ConstantValues.IntakeConstants.MAX_UP_ROTATION

        self.cfg = configs.TalonFXConfiguration()
        self.cfg.motor_output.neutral_mode=signals.NeutralModeValue.COAST
        self.cfg.slot0.k_p = ConstantValues.IntakeConstants.ARM_KP
        self.cfg.slot0.k_d = ConstantValues.IntakeConstants.ARM_KD
        self.cfg.slot0.k_i = ConstantValues.IntakeConstants.ARM_KI
        self.cfg.slot0.gravity_type = GravityTypeValue.ARM_COSINE
        self.cfg.slot0.k_g = ConstantValues.IntakeConstants.ARM_KG
        self.cfg.motion_magic.motion_magic_cruise_velocity=1.5
        self.cfg.motion_magic.motion_magic_acceleration=1

        self.cfg.hardware_limit_switch.forward_limit_source
        self.cfg.torque_current.peak_forward_torque_current = ConstantValues.IntakeConstants.ARM_PEAK_FORWARD_TORQUE_CURRENT
        self.cfg.torque_current.peak_reverse_torque_current = ConstantValues.IntakeConstants.ARM_PEAK_REVERSE_TORQUE_CURRENT
        self.cfg.feedback.sensor_to_mechanism_ratio = ConstantValues.IntakeConstants.SENSOR_TO_MECHANISM_RATIO
        self.cfg.feedback.rotor_to_sensor_ratio=1

#        self.cfg.software_limit_switch.forward_soft_limit_threshold= ConstantValues.IntakeConstants.ARM_FORWARDTHRESH
##        self.cfg.software_limit_switch.reverse_soft_limit_threshold= ConstantValues.IntakeConstants.ARM_REVERSETHRESH
#        self.cfg.software_limit_switch.reverse_soft_limit_enable = True
#        self.cfg.software_limit_switch.forward_soft_limit_enable = True        
        self.arm_motor.configurator.apply(self.cfg)

        
    def __init__(self, intakeMotorID, armMotorID, dioPortUp, dioPortDown):
        """
        Initialize PID constants for motors
        """
        Subsystem.__init__(self)

        self.intake_motor = hardware.TalonFX(intakeMotorID)
        self.arm_motor = hardware.TalonFX(armMotorID)
        self.up_limit_switch = DigitalInput(dioPortUp)
        self.down_limit_switch = DigitalInput(dioPortDown)
        self.arm_position_torque = controls.MotionMagicVoltage(0,True).with_slot(0)        
        self.arm_manualControl = controls.DutyCycleOut(0)
        self.brake = controls.NeutralOut()

        self.current_goal_position = ConstantValues.IntakeConstants.MAX_DOWN_ROTATION  
        self.zeroed = False
        self.setup()


        self.intake_duty = controls.VoltageOut(0)
        self.arm_motor.set_position(0)
        

#        self.current_goal_position = None
    def periodic(self):
        arm_position = self.arm_motor.get_position().value_as_double
        if not self.zeroed:
            if not self.down_limit_switch.get():
                self.arm_motor.set_position(0)
                self.zeroed = True
            elif not self.up_limit_switch.get():
                self.arm_motor.set_position(ConstantValues.IntakeConstants.ARM_FORWARDTHRESH)
                self.zeroed = True

            

        """
        rotation of the arm
        """

        
#        if  self.current_goal_position == self.goal_up and not self.up_limit_switch.get() :
#            self.arm_motor.set_position(ConstantValues.IntakeConstants.ARM_FORWARDTHRESH)
#            self.stop_arm()
            
#        if self.down_limit_switch.get() and self.current_goal_position == self.goal_down:
#            self.stop_arm()

        
        if  self.current_goal_position == self.goal_down and not self.down_limit_switch.get():
            self.stop_arm()
            

        SmartDashboard.putBoolean("Up Limit Switch", self.up_limit_switch.get())
        SmartDashboard.putBoolean("Down Limit Switch", self.down_limit_switch.get())
        SmartDashboard.putNumber("Arm Position", self.arm_motor.get_position().value_as_double) 
        SmartDashboard.putNumber("Arm CL FeedForward", self.arm_motor.get_closed_loop_feed_forward().value_as_double)        
        SmartDashboard.putNumber("Arm CL Output", self.arm_motor.get_closed_loop_output().value_as_double)
        SmartDashboard.putNumber("Arm CL Error", self.arm_motor.get_closed_loop_error().value_as_double)                     
        SmartDashboard.putNumber("Arm CL Target", self.arm_motor.get_closed_loop_reference().value_as_double)  

    def intake(self):
        self.intake_motor.set_control(self.intake_duty.with_output(self.voltage))
    def stop_intake(self):
        self.intake_motor.set_control(self.intake_duty.with_output(0))
    def stop_arm(self):
        self.arm_motor.set_control(self.brake)
    def arm_up(self):
        self.current_goal_position = self.goal_up
        if self.zeroed:
            self.arm_motor.set_control(self.arm_position_torque.with_position(self.current_goal_position))
            self.stop_intake()
    def arm_down(self):
        self.current_goal_position = self.goal_down
        if self.zeroed:
            self.arm_motor.set_control(self.arm_position_torque.with_position(self.current_goal_position))        
        #     self.intake()


    def zero_position(self):
        self.stop_arm()
        self.arm_motor.set_position(0)
