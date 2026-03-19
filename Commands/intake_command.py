from typing import override
import commands2
from subsystems.intake import IntakeSystem

class IntakeCommand(commands2.Command):
    def __init__(self):
        self.intake = IntakeSystem.getInstance()
        self.addRequirements(self.intake)
    @override
    def initialize(self):
        self.intake.arm_down()
        self.intake.intake()
    def execute(self):
        self.intake.start_conveyor()
    @override
    def end(self,interrupted:bool):
        self.intake.arm_up()
        self.intake.stop_arm()
        self.intake.stop_intake()
        self.intake.stop_conveyor()
    @override
    def isFinished(self):
        return False