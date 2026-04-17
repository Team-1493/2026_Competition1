from wpilib import AddressableLED
from commands2 import Subsystem
from enum import Enum
from wpilib import DriverStation, Timer

class Alliance(Enum):
    RED = 0
    BLUE = 1

class Hub(Enum):
    RED = 0
    BLUE = 1
    BOTH = 2
    NONE = 3 
    RED_YELLOW = 4   # last 8s of red period
    BLUE_YELLOW = 5  # last 8s of blue period 
    TRANSITION = 6   

class led_system(Subsystem):
   
    def __init__(self):
        Subsystem.__init__(self)
        self.p = AddressableLED(7)
        self.length = 24
        self.p.setColorOrder(AddressableLED.ColorOrder.kRGB)
        self.p.setLength(self.length)
        self.G = [AddressableLED.LEDData(0,128,0) for _ in range(self.length)]
        self.R = [AddressableLED.LEDData(128,0,0) for _ in range(self.length)]
        self.GY =self.set_green_yellow()
        self.RY = self.set_red_yellow()

        self.p.start()        
        self.p.setData(self.R)

        self.prev_alliance = DriverStation.Alliance.kBlue   
        self.i=0
        self.has_message=False
        self.has_alliance = False
        self.msg=None
        self.al = None
        self.timer = Timer()
#        self.timer.restart()
#        self.timer.start()

    def periodic(self):
        time = Timer.getMatchTime()
 #       if DriverStation.isDisabled(): self.timer.reset()
 #       time =160-self.timer.get() 
        print(time)

        if (not self.has_alliance):
            self.al=DriverStation.getAlliance()
            if self.al is not None:
                self.has_alliance = True


        if not self.has_message:
            self.msg=DriverStation.getGameSpecificMessage()
            if self.msg=="R" or self.msg=="B":
                self.has_message=True
    
        hub = get_active_hub(time,self.msg)

        if self.al == DriverStation.Alliance.kRed:
            if hub == Hub.RED or hub == Hub.BOTH:
                self.a = self.G
            elif hub == Hub.BLUE:
                self.a = self.R
            elif hub == Hub.BLUE_YELLOW:
                self.a = self.RY
            else:
                if time<40:
                    self.a = self.G
                else:
                    self.a = self.GY

        elif self.al == DriverStation.Alliance.kBlue:
            if hub == Hub.BLUE or hub == Hub.BOTH:
                self.a = self.G
            elif hub == Hub.RED:
                self.a = self.R
            elif hub == Hub.RED_YELLOW:
                self.a = self.RY
            else:
                if time<40:
                    self.a = self.G
                else:
                    self.a = self.GY

        else:
            self.a = self.G



        self.p.setData(self.a)
        self.prev_alliance = self.al


    def set_green_yellow(self):
        data = []
        for i in range(24):
            if i%2 ==0:
                data.append(AddressableLED.LEDData(128,128,0))
            else:
                data.append(AddressableLED.LEDData(0,128,0)) 
        return data
    
    def set_red_yellow(self):
        data = []
        for i in range(24):
            if i%2 ==0:
                data.append(AddressableLED.LEDData(128,128,0))
            else:
                data.append(AddressableLED.LEDData(128,0,0)) 
        return data


def get_active_hub(match_time: float, gsm: str) -> Hub:
    """
    match_time: seconds remaining (150 -> 0)
    gsm: "R" or "B" (winner of auto)
    """

    # --- AUTO ---
    if match_time > 140:
        return Hub.BOTH

    # --- TRANSITION ---
    if match_time > 130:
        return Hub.BOTH

    # --- ENDGAME ---
    if match_time <= 30:
        return Hub.BOTH

    # --- DETERMINE LOSER ---
    if gsm == "R":
        loser = Hub.BLUE
        winner = Hub.RED
    elif gsm == "B":
        loser = Hub.RED
        winner = Hub.BLUE
    else:
        return Hub.NONE

    # --- CYCLING PERIOD ---
    time_into_cycle = 130 - match_time  # 0 → 100
    period_index = int(time_into_cycle // 25)  # 0–3
    time_into_period = time_into_cycle % 25

    # Determine active hub
    if period_index % 2 == 0:
        active = loser
    else:
        active = winner

    # --- YELLOW (last 10 seconds) ---
    if time_into_period >= 15 :
        if active == Hub.RED:
            return Hub.RED_YELLOW
        elif active == Hub.BLUE:
            return Hub.BLUE_YELLOW

    return active