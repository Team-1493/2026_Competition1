from typing import override
import commands2
from wpilib import Timer
from math import pi, sin
from Constants1 import ConstantValues
from subsystems.intake import IntakeSystem

class AgitateIntake(commands2.Command):
    def __init__(self):
        self.intake = IntakeSystem.getInstance()
        self.addRequirements(self.intake)
        self.amplitude = ConstantValues.IntakeConstants.AGITATE_AMPLITUDE
        self.b = ConstantValues.IntakeConstants.AGITATE_FREQUENCY * (2 * pi)
    @override
    def initialize(self):
        self.intake.start_conveyor()
        Timer.reset()
        Timer.start()
    def execute(self):
        self.intake.conveyor_voltage = self.amplitude * sin(self.b * Timer.getTimestamp())
    @override
    def end(self,interrupted:bool):
        self.intake.stop_conveyor()
        Timer.reset()
    @override
    def isFinished(self):
        return False