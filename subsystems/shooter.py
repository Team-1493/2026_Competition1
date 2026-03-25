from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog
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
        self.sysid_routine = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                self._run_shooter_sysid,
                self._log_shooter_sysid,
                self,
                "shooter",
            ),
        )



        """
        for motor in self.all_motors:
            motor.get_velocity().set_update_frequency(200)
            motor.get_motor_voltage().set_update_frequency(100)        
            motor.optimize_bus_utilization()
        
        self.feeder_motor.get_velocity().set_update_frequency(100)
        self.feeder2_motor.get_velocity().set_update_frequency(100)
        self.feeder_motor.get_motor_voltage().set_update_frequency(100)
        self.feeder2_motor.get_motor_voltage().set_update_frequency(100)        
        self.feeder_motor.optimize_bus_utilization_for_all([self.feeder2_motor,self.feeder2_motor])
        """

    def periodic(self): 
        pass

    def shoot(self):
        """
        Move the leader motor
        """
        
#        self.velocity = velocity
        self.velocity = SmartDashboard.getNumber('Shooting Velocity', 0)
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
        return sum([abs(m.get_velocity().value_as_double) for m in self.all_motors]) / len(self.all_motors)

    def write_to_dashboard(self):
        self.shooterActualVel = self.leader_motor.get_velocity().value_as_double
        self.feeder1_actual_vel = self.feeder_motor.get_velocity().value_as_double        
        SmartDashboard.putNumber('Leader actual velocity',self.shooterActualVel)
        SmartDashboard.putNumber('Feeder1 actual velocity',self.feeder1_actual_vel)


    def _run_shooter_sysid(self, voltage) -> None:
        voltage_output = getattr(voltage, "value", voltage)
        self.leader_motor.set_control(self.voltage_control.with_output(voltage_output))

    def _log_shooter_sysid(self, log: SysIdRoutineLog) -> None:
        log.motor("shooter_leader").voltage(
            self.leader_motor.get_motor_voltage().value_as_double
        ).angularVelocity(self.leader_motor.get_velocity().value_as_double)

    def shooter_sysid_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sysid_routine.quasistatic(direction)

    def shooter_sysid_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.sysid_routine.dynamic(direction)

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