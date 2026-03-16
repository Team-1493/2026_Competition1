from commands2 import Subsystem
from wpilib import SmartDashboard, DigitalInput
import wpilib
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

        self.cfg = configs.TalonFXConfiguration()
        self.cfg.slot0.k_v = ConstantValues.ShooterConstants.LEADER_KV
        self.cfg.slot0.k_p = ConstantValues.ShooterConstants.LEADER_KP        
        self.leader_motor.configurator.apply(self.cfg)
        self.conveyor_voltage = ConstantValues.ShooterConstants.CONVEYOR_VOLTAGE   

        self.velocity_voltage = controls.VelocityVoltage(0,0)
        self.voltage_control = controls.VoltageOut(0)
        self.brake = controls.NeutralOut()

        SmartDashboard.putNumber('Shooter Velocity', 0)
        SmartDashboard.putNumber('Shooter kP', 0)
        SmartDashboard.putNumber('Shooter kV', 0)
        SmartDashboard.putNumber('Shooter conveyor V', 0)        

        
    def periodic(self): pass
    def shoot(self, velocity):
        """
        Move the leader motor
        """
#        self.velocity = velocity
        self.velocity = SmartDashboard.getNumber('Shooter Velocity', 0)
        self.leader_motor.set_control(self.velocity_voltage.with_velocity(self.velocity))
    def stop_shooter(self):
        self.leader_motor.set_control(self.brake)
        # self.leader_motor.set_control(self.voltage_control.with_output(0))
    def immediate_move_conveyor(self):
        self.feeder_motor.set_control(self.voltage_control.with_output(self.conveyor_voltage))
        self.feeder2_motor.set_control(self.voltage_control.with_output(self.conveyor_voltage))        
    def move_conveyor(self):
        # If the shooter motor's velocity is around the threshold, THEN move the feeder
        if abs((self.velocity * 0.1) - self.mean_shooter_velocity()) <= 0.1 or self.mean_shooter_velocity() >= self.velocity:
            self.feeder_motor.set_control(self.voltage_control.with_output(self.conveyor_voltage))
            self.feeder2_motor.set_control(self.voltage_control.with_output(self.conveyor_voltage))            
    def stop_conveyor(self):
        # self.feeder_motor.set_control(self.voltage_control.with_output(0))
        self.feeder_motor.set_control(self.brake)
        self.feeder2_motor.set_control(self.brake)
        print("!!!!!!!!!!!!!!!!!!!!!  STOPPING CONVEYOR !!!!!!!!!!!!!!!!!!!!!")
    def mean_shooter_velocity(self):
        return sum(abs([m.get_velocity().value for m in self.all_motors])) / len(self.all_motors)



    def update_constants(self):
        self.cfg.slot0.k_v = SmartDashboard.getNumber('Shooter kV', 0)
        self.cfg.slot0.k_p = SmartDashboard.getNumber('Shooter kP', 0)
        self.leader_motor.configurator.apply(self.cfg)
        self.conveyor_voltage = SmartDashboard.getNumber('Shooter conveyor V', 0)   