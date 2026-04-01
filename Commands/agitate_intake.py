from typing import override
import commands2
from wpilib import Timer, SmartDashboard
from math import pi, sin
from Constants1 import ConstantValues
from subsystems.intake import IntakeSystem

class AgitateIntake(commands2.Command):
    def __init__(self):
        self.timer = Timer()
        self.intake = IntakeSystem.getInstance()
        self.addRequirements(self.intake)
#        self.amplitude = ConstantValues.IntakeConstants.AGITATE_AMPLITUDE
#        self.b = ConstantValues.IntakeConstants.AGITATE_FREQUENCY * (2 * pi)
    @override
    def initialize(self):
        self.amplitude = 2
        self.b =3 * (2 * pi)

        # self.intake.start_conveyor()
        self.timer.reset()
        self.timer.start()
    def execute(self):
        SmartDashboard.putNumber("Agitate bt", sin(self.b * self.timer.get()))
        self.intake.conveyor_voltage = self.amplitude * sin(self.b * self.timer.get())
        self.intake.start_conveyor()
    @override
    def end(self,interrupted:bool):
        self.intake.stop_conveyor()
        self.timer.reset()
    @override
    def isFinished(self):
        return False