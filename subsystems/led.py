from wpilib import AddressableLED
from commands2 import Subsystem
import random
from wpilib import DriverStation
class led_system(Subsystem):
   
    def __init__(self):
        Subsystem.__init__(self)
        self.p = AddressableLED(7)
        self.p.setColorOrder(AddressableLED.ColorOrder.kRGB)
        self.p.setLength(50)
        self.G=[AddressableLED.LEDData(0,128,0)]*50        
        self.R=[AddressableLED.LEDData(128,0,0)]*50
        self.p.start()        
        self.p.setData(self.R)

        self.prev_alliance = DriverStation.Alliance.kBlue   
        self.i=0
    def periodic(self):
        mt=DriverStation.getMatchTime()
#        msg=DriverStation.getGameSpecificMessage()[0]
        msg = "R"
        al=DriverStation.getAlliance()
        print(al,"  ",self.prev_alliance,"  ",msg)
       
        if msg=="R":
            if al==DriverStation.Alliance.kRed:
                self.a=self.G
            else:
                self.a=self.R
        if msg=="B":
            if al==DriverStation.Alliance.kBlue:
                self.a=self.G
            else:
                self.a=self.R
        if al != self.prev_alliance:
            print("**************** SETTING LED")                
            self.p.setData(self.a)
#            self.p.start()
        self.prev_alliance = al