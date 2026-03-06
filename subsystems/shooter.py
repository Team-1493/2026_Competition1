from commands2 import Subsystem
from wpilib import SmartDashboard, DigitalInput
import wpilib
from phoenix6 import hardware, configs, controls,signals
from phoenix6.signals import GravityTypeValue
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
    def __init__(self, leader_motor_id: int, follower_ids: list[int], opposite_follower_ids: list[int]):
        """
        Initialize PID constants for motors
        """
        Subsystem.__init__(self)
        self.leader_motor = hardware.TalonFX(leader_motor_id)

        self.followers, self.opposite_followers = [hardware.TalonFX(m) for m in follower_ids], [hardware.TalonFX(m) for m in opposite_follower_ids]

        for motor in self.followers:
            motor.set_control(controls.Follower(leader_id=leader_motor_id))
        for motor in self.opposite_followers:
            motor.set_control(controls.Follower(leader_id=leader_motor_id, oppose_master_direction=True))
        self.cfg = configs.TalonFXConfiguration()
        self.cfg.slot0.k_p = ConstantValues.ShooterConstants.LEADER_KP
        self.cfg.slot0.k_v = ConstantValues.ShooterConstants.LEADER_KV

        self.leader_motor.configurator.apply(self.cfg)
        
        self.velocity = ConstantValues.ShooterConstants.VELOCITY

        self.torque_current = controls.VelocityTorqueCurrentFOC(velocity=self.velocity)

        self.brake = controls.NeutralOut()
    def shoot(self):
        """
        Move the leader motor
        """
        self.leader_motor.set_control(self.torque_current.with_velocity(self.velocity))
    def stop_shooter(self):
        self.leader_motor.set_control(self.brake)
        
        