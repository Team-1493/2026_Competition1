import math
from typing import override
import commands2
from wpilib import SmartDashboard
from subsystems.intake import IntakeSystem

class IntakeStop(commands2.Command):
    def __init__(self):
        self.intake = IntakeSystem.getInstance()

    @override
    def initialize(self):
        SmartDashboard.putString("Intake State", "STOPPING")                                   
        self.intake.stop_intake()
        self.intake.stop_conveyor()        

    def execute(self):
        pass

    @override
    def end(self,interrupted:bool):
        pass

    @override
    def isFinished(self):
        return True
