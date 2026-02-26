import math
from typing import override
import commands2
from wpilib import SmartDashboard, Timer
import wpimath
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from phoenix6 import swerve
from  subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain

class FindSlipCurrent(commands2.Command):
    def __init__(self):
        self.drivetrain = DrivetrainGenerator.getInstance()

      
        self.addRequirements(self.drivetrain)
        self.timer = Timer()
        self.voltageRate = 0.01  #ramp rate volts per second
        self.driveKS = swerve.requests.SysIdSwerveTranslation()
        
    @override
    def initialize(self):
        self.timeCurrent = 0
        self.voltage = 0
        self.timer.reset()
        self.timer.start()
        self.timePrevious = self.timer.get()
        self.kS = 0
        self.avgVolt=0
        self.avgTorqueCurrnet=0    
        self.maxV = 0
        self.maxTC=0    

        SmartDashboard.putNumber("Slip - MaxV", self.maxV) 
        SmartDashboard.putNumber("Slip - MaxTC", self.maxTC)    


    def execute(self) -> None:
        timeCurrent = self.timer.get()
        deltaTime = timeCurrent - self.timePrevious
        self.voltage += deltaTime*self.voltageRate


        self.drivetrain.set_control(self.driveKS.with_volts(self.voltage))
        chassisSpeed = self.drivetrain.get_state().speeds
        self.spd = math.sqrt(chassisSpeed.vx**2+chassisSpeed.vy**2)
  

  
        mod0 = self.get_torque_current(0)
        mod1 = self.get_torque_current(1)
        mod2 = self.get_torque_current(2)
        mod3 = self.get_torque_current(3)
            
        volt0 = self.get_voltage(0)
        volt1 = self.get_voltage(1)
        volt2 = self.get_voltage(2)
        volt3 = self.get_voltage(3)

        self.avgVolt=(volt0+volt1+volt2+volt3)/4
        self.avgTorqueCurrnet = (mod0+mod1+mod2+mod3)/4

        if self.avgVolt>self.maxV: self.maxV=self.avgVolt
        if self.avgTorqueCurrnet>self.maxTC: self.maxTC=self.avgTorqueCurrnet

        SmartDashboard.putNumber("Slip - MaxV", self.maxV) 
        SmartDashboard.putNumber("Slip - MaxTC", self.maxTC)         

    @override
    def end(self, interrupted: bool) :
        self.drivetrain.set_control(self.driveKS.with_volts(0))

    @override
    def isFinished(self):
        return self.spd>0.25
    

        
    def get_voltage(self,module):
        return abs(self.drivetrain.get_module(module).drive_motor.get_motor_voltage().value_as_double)
    
    def get_torque_current(self,module):
        return abs(self.drivetrain.get_module(module).drive_motor.get_torque_current().value_as_double)
