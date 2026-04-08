from wpilib import AddressableLED
from commands2 import Subsystem
import random

class led_system(Subsystem):
    
    def __init__(self):
        Subsystem.__init__(self)
        self.p = AddressableLED(1)
        self.p.setColorOrder(AddressableLED.ColorOrder.kRGB)
        self.p.setLength(50)
        self.G=[AddressableLED.LEDData(0,128,0)]*50
        self.R=[AddressableLED.LEDData(128,0,0)]*50
        self.a=[AddressableLED.LEDData(0,255,255)]*50
        self.p.setData(self.a)
        
        self.p.start()
        self.i=0
    def periodic(self):
        
        if (self.i>1):
            self.x=random.randint(0,49)
            self.a[self.x]=AddressableLED.LEDData(random.randint(0,255),random.randint(0,255),random.randint(0,255))
            self.p.setData(self.a)
            self.i=0
        self.i =self.i+1
        