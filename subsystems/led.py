from wpilib import AddressableLED
from commands2 import Subsystem
import random
from wpilib import DriverStation
class led_system(Subsystem):
   
    def __init__(self):
        Subsystem.__init__(self)
        self.p = AddressableLED(1)
        self.p.setColorOrder(AddressableLED.ColorOrder.kRGB)
        self.p.setLength(50)
        self.G=[AddressableLED.LEDData(0,128,0)]*50
        self.R=[AddressableLED.LEDData(256,128,0)]*50
        self.p.setData(self.a)
        self.p.start()
        self.i=0
    def periodic(self):
        mt=DriverStation.getMatchTime()
        al=DriverStation.getAlliance
        if al=="red":
            al="R"
        elif al=="blue":
            al="B"
        msg=DriverStation.getGameSpecificMessage()[0]
       
        if msg=="R":
            if al=="R":
                self.a=self.G
            else:
                self.a=self.R
        if msg=="B":
            if al=="B":
                self.a=self.G
            else:
                self.a=self.R
        self.p.setData(self.a)
        self.i =self.i+1