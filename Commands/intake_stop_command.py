import math
from typing import override
import commands2
from wpilib import SmartDashboard, Timer
from subsystems.shooter import ShooterSystem
from subsystems.intake import IntakeSystem
from Utilities.helper_methods import HelperMethods

class IntakeStop(commands2.Command):
    def __init__(self):
        self.intake = IntakeSystem.getInstance()
        self.addRequirements(self.intake)

    @override
    def initialize(self):
        SmartDashboard.putString("Intake State", "STOPPING")                                   
        self.intake.intake()

    def execute(self):
        pass

    @override
    def end(self,interrupted:bool):
        pass

    @override
    def isFinished(self):
        return True
