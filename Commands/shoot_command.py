from typing import override
import commands2
from wpilib import SmartDashboard, Timer
from subsystems.shooter import ShooterSystem
from subsystems.intake import IntakeSystem

class ShootCommand(commands2.Command):
    def __init__(self):
        self.shooter = ShooterSystem.getInstance()
        self.intake = IntakeSystem.getInstance()
        self.addRequirements(self.shooter,self.intake)
        self.timer = Timer()
    @override
    def initialize(self):
#        self.intake.arm_up()
        self.intake.start_conveyor()
        self.shooter.shoot()
        self.timer.reset()
        self.timer.start()

    def execute(self):
        self.shooter.move_conveyor()
        if self.timer.get()<0.75:
            self.intake.arm_wag_up()
            SmartDashboard.putNumber("Wag", 1) 
        elif self.timer.get()<1.5:
            self.intake.arm_down()
            SmartDashboard.putNumber("Wag", 0) 
        else:
            self.timer.restart()

    @override
    def end(self,interrupted:bool):
        self.intake.stop_arm()
        self.intake.stop_conveyor()
        self.shooter.stop_conveyor()
        self.shooter.stop_shooter()
    @override
    def isFinished(self):
        return False