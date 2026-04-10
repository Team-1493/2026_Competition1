import math
from typing import override
import commands2
from wpilib import SmartDashboard
from subsystems.intake import IntakeSystem

class IntakeStartAuto(commands2.Command):
    def __init__(self):
        self.intake = IntakeSystem.getInstance()
        self.addRequirements(self.intake)

    @override
    def initialize(self):
        self.intake.intake_auto()

    def execute(self):
            pass

    @override
    def end(self,interrupted:bool):
        pass

    @override
    def isFinished(self):
        return True
