from commands2 import Command, Subsystem
from wpilib import SmartDashboard
from phoenix6 import hardware, configs, controls
from phoenix6.signals import MotorAlignmentValue
from Constants1 import ConstantValues

class ShooterSystem(Subsystem):
    instance=None
    
    @staticmethod
    def getInstance():
        if ShooterSystem.instance == None:
            ShooterSystem.instance = ShooterSystem(14, [15], [16, 17], 12,13)
            print('*' * 22 + ' SHOOTER ' + '*' * 22)
        return ShooterSystem.instance
    def setup(self):
        pass
    def __init__(self,
                leader_motor_id: int,
                follower_ids: list[int],
                opposite_follower_ids: list[int],
                feeder_id: int,
                feeder2_id: int
                ) -> None:
        """
        Initialize PID constants for motors
        """
        Subsystem.__init__(self)
        "sam was here"

        self.leader_motor = hardware.TalonFX(leader_motor_id)
        self.feeder_motor = hardware.TalonFX(feeder_id)
        self.feeder2_motor = hardware.TalonFX(feeder2_id)        

        self.followers, self.opposite_followers = [hardware.TalonFX(m) for m in follower_ids], [hardware.TalonFX(m) for m in opposite_follower_ids]

        for motor in self.followers:
            motor.set_control(controls.Follower(leader_motor_id, motor_alignment=MotorAlignmentValue.ALIGNED))
        for motor in self.opposite_followers:
            motor.set_control(controls.Follower(leader_motor_id, motor_alignment=MotorAlignmentValue.OPPOSED))
        self.all_motors = self.followers + self.opposite_followers + [self.leader_motor]

        self.leader_cfg = configs.TalonFXConfiguration()
        self.leader_cfg.slot0.k_v = ConstantValues.ShooterConstants.LEADER_KV
        self.leader_cfg.slot0.k_p = ConstantValues.ShooterConstants.LEADER_KP
        self.leader_cfg.slot0.k_s = ConstantValues.ShooterConstants.LEADER_KS 
        self.leader_cfg.current_limits.stator_current_limit_enable=True
        self.leader_cfg.current_limits.stator_current_limit=100                
        self.leader_motor.configurator.apply(self.leader_cfg)

        self.feeder_cfg = configs.TalonFXConfiguration()
        self.feeder_cfg.slot0.k_v = ConstantValues.ShooterConstants.FEEDER_KV
        self.feeder_cfg.slot0.k_p = ConstantValues.ShooterConstants.FEEDER_KP
        
        self.feeder_motor.configurator.apply(self.feeder_cfg)
        self.feeder2_motor.configurator.apply(self.feeder_cfg)

        self.velocity_voltage = controls.VelocityVoltage(0,0)
        self.voltage_control = controls.VoltageOut(0)
        self.brake = controls.NeutralOut()

        self.conveyor_velocity = ConstantValues.ShooterConstants.CONVEYOR_VELOCITY
        self.velocity = ConstantValues.ShooterConstants.SHOOTING_VELOCITY


        self.leader_motor.get_velocity().set_update_frequency(200)
        for motors in self.followers:
            motor.get_velocity().set_update_frequency(200)

        for motor in self.opposite_followers:
            motor.get_velocity().set_update_frequency(200)

        for motor in self.all_motors:
            motor.optimize_bus_utilization()



    def periodic(self): 
        pass

    def shoot(self,vel):
        """
        Move the leader motor
        """
        
        self.velocity = vel
        self.leader_motor.set_control(self.velocity_voltage.with_velocity(self.velocity))
  
    def stop_shooter(self):
        self.leader_motor.set_control(self.brake)
        # self.leader_motor.set_control(self.voltage_control.with_output(0))
  
    def immediate_move_conveyor(self):
        self.feeder_motor.set_control(self.velocity_voltage.with_velocity(self.conveyor_velocity))
        self.feeder2_motor.set_control(self.velocity_voltage.with_velocity(self.conveyor_velocity))        
  
    def move_conveyor(self):
        # If the shooter motor's velocity is around the threshold, THEN move the feeder
        mean =  self.mean_shooter_velocity()
#        SmartDashboard.putNumber("ShooterCheck",abs(self.velocity  - mean)/ (mean+0.0001))
        if abs(self.velocity  - mean)/ (mean+0.0001) <= 0.08 or mean >= self.velocity * 0.98:
            self.feeder_motor.set_control(self.velocity_voltage.with_velocity(self.conveyor_velocity))
            self.feeder2_motor.set_control(self.velocity_voltage.with_velocity(self.conveyor_velocity))            
  
    def stop_conveyor(self):
        # self.feeder_motor.set_control(self.voltage_control.with_output(0))
        self.feeder_motor.set_control(self.brake)
        self.feeder2_motor.set_control(self.brake)
  
    def mean_shooter_velocity(self):
        return sum([abs(m.get_velocity().value_as_double) for m in self.all_motors]) / 4.

    def write_to_dashboard(self):
        SmartDashboard.putNumber('Shooter mean velocity',self.mean_shooter_velocity())


    def update_constants(self):
        self.leader_cfg.slot0.k_v = SmartDashboard.getNumber('Leader KV', 0)
        self.leader_cfg.slot0.k_p = SmartDashboard.getNumber('Leader KP', 0)
        self.leader_cfg.slot0.k_s = ConstantValues.ShooterConstants.LEADER_KS         
        self.leader_motor.configurator.apply(self.leader_cfg)
        self.feeder_cfg.slot0.k_p = SmartDashboard.getNumber('Feeder KP', 0)
        self.feeder_cfg.slot0.k_v = SmartDashboard.getNumber('Feeder KV', 0)
        self.feeder_motor.configurator.apply(self.feeder_cfg)
        self.feeder2_motor.configurator.apply(self.feeder_cfg)

        self.conveyor_velocity = SmartDashboard.getNumber('Conveyor Velocity', 0)
        self.conveyor_voltage = SmartDashboard.getNumber('Shooter conveyor V', 0)

