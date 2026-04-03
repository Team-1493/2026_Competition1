import math
from typing import override
import commands2
from wpilib import SmartDashboard, Timer
from subsystems.shooter import ShooterSystem
from subsystems.intake import IntakeSystem
from Utilities.helper_methods import HelperMethods

class ShootCommand(commands2.Command):
    def __init__(self):
        SmartDashboard.putNumber("Shooter Speed Calculated",0)   
        SmartDashboard.putNumber("Shooter Speed SF",1)            
        SmartDashboard.putString("Shoot State", "XXX")                            
        self.shooter = ShooterSystem.getInstance()
        self.intake = IntakeSystem.getInstance()
        self.timer = Timer()
        self.addRequirements(self.shooter)#,self.intake)

    @override
    def initialize(self):
        shoot_speed = self.get_shooter_speed()
#        self.intake.arm_up()
        SmartDashboard.putNumber("Shooter Speed Calculated",shoot_speed)        
        self.shooter.shoot(shoot_speed)
        self.intake.start_conveyor_reverse()
        self.timer.reset()
        self.timer.start()
        SmartDashboard.putString("Shoot State", "SHOOTING")                                   

    def execute(self):
        curretTime = self.timer.get()
        self.shooter.move_conveyor()
        if curretTime>0.05 and curretTime<1:
            self.intake.stop_conveyor()

        if self.timer.get()>1:
            self.intake.start_conveyor()            
#            self.intake.arm_to_position(0.1 + 0.1*math.sin(1.5*self.timer.get()*math.pi) )
            pos = min(0.18,0.18*self.timer.get()/4.5)
            self.intake.arm_to_position(pos )

    @override
    def end(self,interrupted:bool):
        SmartDashboard.putString("Shoot State", "NOT")                                   
        self.intake.stop_arm()
        self.intake.stop_conveyor()
        self.shooter.stop_conveyor()
        self.shooter.stop_shooter()
    @override
    def isFinished(self):
        return False
    
    def get_shooter_speed(self):
        shooter_speed= SmartDashboard.getNumber('Shooting Velocity', 0)
        if shooter_speed<0: 
             shooter_speed = HelperMethods.calculate_shoot_speed()
             shooter_speed=shooter_speed*SmartDashboard.getNumber("Shooter Speed SF",1)
        SmartDashboard.putNumber("Shooter Speed Calculated",shooter_speed)        

        return shooter_speed
