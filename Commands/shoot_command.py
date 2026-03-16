from typing import override
import commands2
from subsystems.shooter import ShooterSystem
from subsystems.intake import IntakeSystem

class ShootCommand(commands2.Command):
    def __init__(self):
        self.shooter = ShooterSystem.getInstance()
        self.intake = IntakeSystem.getInstance()
        self.addRequirements(self.shooter)
        self.addRequirements(self.intake)
    @override
    def initialize(self, velocity):
        self.intake.arm_up()
        self.intake.intake()
        self.shooter.shoot(velocity=velocity)
    def execute(self):
        self.shooter.move_conveyor()
    @override
    def end(self):
        self.intake.stop_intake()
        self.intake.stop_arm()
        self.shooter.stop_conveyor()
        self.shooter.stop_shooter()
    @override
    def isFinished(self):
        return False