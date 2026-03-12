from commands2 import Subsystem
from wpilib import SmartDashboard, DigitalInput
import wpilib
from phoenix6 import hardware, configs, controls
from phoenix6.signals import MotorAlignmentValue
from Constants1 import ConstantValues

class Shooter(Subsystem):
    instance=None
    
    @staticmethod
    def getInstance():
        if Shooter.instance == None:
            Shooter.instance = Shooter()
            print('*' * 22 + ' SHOOTER ' + '*' * 22)
        return Shooter.instance
    def setup(self):
        pass
    def __init__(self,
                leader_motor_id: int,
                follower_ids: list[int],
                opposite_follower_ids: list[int],
                feeder_id: int):
        """
        Initialize PID constants for motors
        """
        Subsystem.__init__(self)

        self.leader_motor = hardware.TalonFX(leader_motor_id)
        self.feeder_motor = hardware.TalonFX(feeder_id)

        self.followers, self.opposite_followers = [hardware.TalonFX(m) for m in follower_ids], [hardware.TalonFX(m) for m in opposite_follower_ids]

        for motor in self.followers:
            motor.set_control(controls.Follower(leader_id=leader_motor_id, motor_alignment=MotorAlignmentValue(0)))
        for motor in self.opposite_followers:
            motor.set_control(controls.Follower(leader_id=leader_motor_id, motor_alignment=MotorAlignmentValue(1)))
        self.cfg = configs.TalonFXConfiguration()
        self.cfg.slot0.k_p = ConstantValues.ShooterConstants.LEADER_KP
        self.cfg.slot0.k_v = ConstantValues.ShooterConstants.LEADER_KV

        self.leader_motor.configurator.apply(self.cfg)
        
        self.velocity = ConstantValues.ShooterConstants.VELOCITY
        self.threshold = ConstantValues.ShooterConstants.THRESHOLD
        self.torque_current = controls.VelocityTorqueCurrentFOC(velocity=self.velocity)
        self.voltage_control = controls.VoltageOut(0)
        self.brake = controls.NeutralOut()
    def periodic(self, voltage):
        # If the shooter motor's velocity is around the threshold, THEN move the feeder
        if abs(self.threshold - abs(self.leader_motor.get_velocity().value_as_double)) <= 0.1:
            self.feeder_motor.set_control(self.voltage_control.with_output(voltage))
    def shoot(self, velocity):
        """
        Move the leader motor
        """
        self.leader_motor.set_control(self.torque_current.with_velocity(velocity))
        self.velocity = velocity
    def stop_shooter(self):
        self.leader_motor.set_control(self.brake)
        
        