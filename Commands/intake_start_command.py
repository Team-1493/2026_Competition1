import math
from typing import override
import commands2
from wpilib import SmartDashboard
from subsystems.intake import IntakeSystem

class IntakeStart(commands2.Command):
    def __init__(self):
        SmartDashboard.putString("Intake State", "XXX")                            
        self.intake = IntakeSystem.getInstance()

    @override
    def initialize(self):
        SmartDashboard.putString("Intake State", "STARTING")                                   
        self.intake.intake()

    def execute(self):
            pass

    @override
    def end(self,interrupted:bool):
        pass

    @override
    def isFinished(self):
        return True
