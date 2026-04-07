from wpilib import AddressableLED
from commands2 import Subsystem
import random

class led_system(Subsystem):
    
    def __init__(self):
        Subsystem.__init__(self)
        self.p = AddressableLED(1)
        self.p.setColorOrder(AddressableLED.ColorOrder.kRGB)
        self.p.setLength(100)
        self.a=[AddressableLED.LEDData(0,255,255)]*100
        self.p.setData(self.a)
        
        self.p.start()

    def periodic(self):
        self.x=random.randint(0,99)
        AddressableLED.LEDData.setRGB(self.a[self.x],[random.randint(0,255),random.randint(0,255),random.randint(0,255)])
        #self.p.setData(self.a)
        pass