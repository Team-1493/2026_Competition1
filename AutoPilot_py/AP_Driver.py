# all distances in meters, all angles in radians

from AutoPilot_py.AP import Autopilot
from AutoPilot_py.Contraints import APConstraints
from AutoPilot_py.Profile import APProfile

class ap_driver:
    instance=None
    
    @staticmethod
    def getInstance():
        if ap_driver.instance == None:
            ap_driver.instance = ap_driver()
            print("**********************  AP Driver Created  **********************") 
        return ap_driver.instance


    def __init__(self):        
        self.kConstraints = APConstraints().with_acceleration(8.0).with_jerk(8.0).with_velocity(3)
        self.kProfile = (
            APProfile(self.kConstraints).
            with_ErrorXY(0.06).
            with_ErrorTheta(0.05).
            with_BeelineRadius(0.2)
        )
        self.kAutopilot = Autopilot(self.kProfile)