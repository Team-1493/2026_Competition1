from typing import override
import commands2
from subsystems.shooter import ShooterSystem
from subsystems.intake import IntakeSystem

class IntakeAndShoot(commands2.Command):
    def __init__(self):
        self.shooter = ShooterSystem.getInstance()
        self.intake = IntakeSystem.getInstance()
        self.addRequirements(self.shooter)
    @override
    def initialize(self):
        self.intake.arm_up()
        self.intake.intake()
        self.shooter.shoot()
    def execute(self):
        self.shooter.move_conveyor()
    @override
    def end(self,interrupted:bool):
        self.intake.stop_intake()
        self.intake.stop_arm()
        self.shooter.stop_conveyor()
        self.shooter.stop_shooter()
    @override
    def isFinished(self):
        return False