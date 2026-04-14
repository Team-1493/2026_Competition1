from commands2 import Subsystem
from wpilib import SmartDashboard, DigitalInput
import wpilib
from phoenix6 import BaseStatusSignal, hardware, configs, controls,signals
from phoenix6.signals import MotorAlignmentValue,NeutralModeValue
from phoenix6.signals import GravityTypeValue
from Constants1 import ConstantValues

class IntakeSystem(Subsystem):
    instance=None
    
    @staticmethod
    def getInstance():
        if IntakeSystem.instance == None:
            IntakeSystem.instance = IntakeSystem(10,18,9,11,8,9)
            print('*' * 22 + ' INTAKE ' + '*' * 22)
        return IntakeSystem.instance
    def __init__(self, intakeMotorID, intakeFollowerMotorID,armMotorID, conveyorMotorID, dioPortUp, dioPortDown):
        """
        Initialize PID constants for motors
        """
        Subsystem.__init__(self)
        self.zeroed = False
        self.intake_motor = hardware.TalonFX(intakeMotorID)
        self.intake_follower_motor = hardware.TalonFX(intakeFollowerMotorID) 
#        self.intake_follower_motor.set_control(controls.Follower(intakeMotorID, motor_alignment=MotorAlignmentValue.OPPOSED))               
        self.arm_motor = hardware.TalonFX(armMotorID)
        self.conveyor_motor = hardware.TalonFX(conveyorMotorID)
        self.up_limit_switch = DigitalInput(dioPortUp)
        self.down_limit_switch = DigitalInput(dioPortDown)
        self.arm_position_torque = controls.MotionMagicVoltage(0,True).with_slot(0)        
        self.arm_manualControl = controls.DutyCycleOut(0)
        self.brake = controls.NeutralOut()

        self.current_goal_position = ConstantValues.IntakeConstants.MAX_DOWN_ROTATION
        self.conveyor_voltage = ConstantValues.IntakeConstants.INTAKE_CONVEYOR_VOLTAGE
        self.conveyor_slow_voltage = ConstantValues.IntakeConstants.INTAKE_CONVEYOR_SLOW_VOLTAGE
        self.pos_signal = self.arm_motor.get_position()
        self.vel_signal = self.arm_motor.get_velocity()
        self.signals=[self.pos_signal,self.vel_signal]
        self.intake_actual_SC = 0 
        self.intake_actual_V = 0
        self.intakeFollower_actual_SC = 0 
        self.intakeFollower_actual_V = 0  
        SmartDashboard.putNumber("IntakeMoter velTC",20)
        SmartDashboard.putNumber("IntakeMoter kP",1)                
        SmartDashboard.putNumber("IntakeMoter SC act",self.intake_actual_SC)
        SmartDashboard.putNumber("IntakeMoter V act",self.intake_actual_V)                      
        SmartDashboard.putNumber("IntakeFollower SC act",self.intakeFollower_actual_SC)
        SmartDashboard.putNumber("IntakeFollower V act",self.intakeFollower_actual_V)
    
        """"
        self.arm_motor.get_position().set_update_frequency(200)
        self.arm_motor.get_velocity().set_update_frequency(200)
        self.arm_motor.get_motor_voltage().set_
        te_frequency(200)
        self.arm_motor.get_closed_loop_error().set_update_frequency(200)
        self.arm_motor.get_closed_loop_reference().set_update_frequency(200)
        self.arm_motor.get_motion_magic_at_target().set_update_frequency(200)
        self.arm_motor.get_motion_magic_is_running().set_update_frequency(200)                                
        self.arm_motor.optimize_bus_utilization()
        self.conveyor_motor.get_motor_voltage().set_update_frequency(50)
        self.conveyor_motor.optimize_bus_utilization()                

        """

#        self.intake_motor.get_motor_voltage().set_update_frequency(200)
#        self.intake_motor.optimize_bus_utilization()


        self.voltage_out = controls.VoltageOut(0)
        self.intake_vel = controls.VelocityTorqueCurrentFOC(0)
        self.arm_motor.set_position(0)


        self.setup()

        

#        self.current_goal_position = None

    def periodic(self):

        BaseStatusSignal.refresh_all(self.signals)

        self.arm_position = self.pos_signal.value_as_double
        self.arm_velocity = self.vel_signal.value_as_double      
        self.lsd = not self.down_limit_switch.get()
        self.lsu = not self.up_limit_switch.get()          
        if not self.zeroed:
            if self.lsd:
                self.arm_motor.set_position(0)
                self.zeroed = True
            elif not self.lsu:
                self.arm_motor.set_position(ConstantValues.IntakeConstants.ARM_FORWARDTHRESH)
                self.zeroed = True

#        if  self.current_goal_position == self.goal_up and not self.up_limit_switch.get() :
#            self.arm_motor.set_position(ConstantValues.IntakeConstants.ARM_FORWARDTHRESH)
#            self.stop_arm()
        if  self.current_goal_position == self.goal_down and self.lsd:
            if self.arm_velocity !=0:
                self.stop_arm()
            # continuously rezero the arm position when arm is at the down position
            # (in case the chain slips or something happens to throw off the position)
            if self.arm_velocity==0 and self.arm_position != 0:
                self.arm_motor.set_position(0)

    
    def intake(self):
        self.intake_motor.set_control(self.voltage_out.with_output(ConstantValues.IntakeConstants.INTAKE_VOLTAGE))
        self.intake_follower_motor.set_control(self.voltage_out.with_output(-ConstantValues.IntakeConstants.INTAKE_VOLTAGE))

    def intake_auto(self):
        self.intake_motor.set_control(self.voltage_out.with_output(ConstantValues.IntakeConstants.INTAKE_AUTO_VOLTAGE))
        self.intake_follower_motor.set_control(self.voltage_out.with_output(-ConstantValues.IntakeConstants.INTAKE_AUTO_VOLTAGE))        

    def stop_intake(self):
        self.intake_motor.set_control(self.brake)
        self.intake_follower_motor.set_control(self.brake)

    def stop_arm(self):
        self.arm_motor.set_control(self.brake)

    def start_conveyor(self):
        self.conveyor_motor.set_control(self.voltage_out.with_output(self.conveyor_voltage))

    def start_conveyor_slow(self):
        self.conveyor_motor.set_control(self.voltage_out.with_output(self.conveyor_slow_voltage))


    def start_conveyor_reverse(self):
        self.conveyor_motor.set_control(self.voltage_out.with_output(4))

    def stop_conveyor(self):
        self.conveyor_motor.set_control(self.brake)

    def arm_up(self):
        self.current_goal_position = self.goal_up
        self.arm_motor.set_control(self.arm_position_torque.with_position(self.current_goal_position))


    def arm_down(self):
        self.current_goal_position = self.goal_down
        self.arm_motor.set_control(self.arm_position_torque.with_position(self.current_goal_position))        


    def arm_wag_up(self):
        self.current_goal_position = 0.07 
        self.arm_motor.set_control(self.arm_position_torque.with_position(self.current_goal_position))        

    def arm_to_position(self, position: float):
        self.current_goal_position = position 
        self.arm_motor.set_control(self.arm_position_torque.with_position(self.current_goal_position))        

    def arm_manualUp(self):
        self.arm_motor.set_control(self.arm_manualControl.with_output(.08)) 

    def arm_manualDown(self):
        self.arm_motor.set_control(self.arm_manualControl.with_output(-.08))    

    def zero_position(self):
        self.stop_arm()
        self.arm_motor.set_position(0)

    def write_to_dashboard(self):
        SmartDashboard.putBoolean("Up Limit Switch", self.lsu)
        SmartDashboard.putBoolean("Down Limit Switch", self.lsd)
        SmartDashboard.putNumber("Arm Position", self.arm_position)
        
        
    def setup(self):
        self.goal_down = ConstantValues.IntakeConstants.MAX_DOWN_ROTATION
        self.goal_up = ConstantValues.IntakeConstants.MAX_UP_ROTATION

        self.cfg = configs.TalonFXConfiguration()
#        self.cfg.motor_output.neutral_mode=signals.NeutralModeValue.COAST
        self.cfg.slot0.k_p = ConstantValues.IntakeConstants.ARM_KP
        self.cfg.slot0.k_d = ConstantValues.IntakeConstants.ARM_KD
        self.cfg.slot0.k_i = ConstantValues.IntakeConstants.ARM_KI
        self.cfg.slot0.gravity_type = GravityTypeValue.ARM_COSINE
        self.cfg.slot0.k_g = ConstantValues.IntakeConstants.ARM_KG
        self.cfg.motion_magic.motion_magic_cruise_velocity=1.5
        self.cfg.motion_magic.motion_magic_acceleration=1
        self.cfg.torque_current.peak_forward_torque_current = ConstantValues.IntakeConstants.ARM_PEAK_FORWARD_TORQUE_CURRENT
        self.cfg.torque_current.peak_reverse_torque_current = ConstantValues.IntakeConstants.ARM_PEAK_REVERSE_TORQUE_CURRENT
        self.cfg.feedback.sensor_to_mechanism_ratio = ConstantValues.IntakeConstants.SENSOR_TO_MECHANISM_RATIO
        self.cfg.feedback.rotor_to_sensor_ratio=1
#        self.cfg.software_limit_switch.forward_soft_limit_threshold= ConstantValues.IntakeConstants.ARM_FORWARDTHRESH
#        self.cfg.software_limit_switch.reverse_soft_limit_threshold= ConstantValues.IntakeConstants.ARM_REVERSETHRESH
#        self.cfg.software_limit_switch.reverse_soft_limit_enable = True
#        self.cfg.software_limit_switch.forward_soft_limit_enable = True        
        self.arm_motor.configurator.apply(self.cfg)


        cfgIntake = configs.TalonFXConfiguration()
        cfgIntake.motor_output.neutral_mode=NeutralModeValue.COAST
        cfgIntake.current_limits.stator_current_limit=100
        cfgIntake.current_limits.with_stator_current_limit_enable(True)
        cfgIntake.current_limits.supply_current_limit=100
        cfgIntake.current_limits.with_supply_current_limit_enable(True)
        cfgIntake.current_limits.supply_current_lower_limit=90
        cfgIntake.current_limits.supply_current_lower_time=.25 

        cfgIntake.slot0.kP=SmartDashboard.getNumber("IntakeMoter kP",1)    
        
        self.intake_motor.configurator.apply(cfgIntake)
        self.intake_follower_motor.configurator.apply(cfgIntake)        


