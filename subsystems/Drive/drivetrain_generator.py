from phoenix6.hardware import TalonFX,TalonFXS
from phoenix6 import configs
from phoenix6.configs import Slot0Configs
from phoenix6 import swerve

from Constants1 import ConstantValues
from subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain
from generated.tuner_constants import TunerConstants

class DrivetrainGenerator():
    instance : CommandSwerveDrivetrain
    instance = None
    

    @staticmethod
    def getInstance():
        "****************** Starting  *****************"
        if DrivetrainGenerator.instance == None:
            DrivetrainGenerator.instance = TunerConstants.create_drivetrain()
            print("**********************  CREATING DT  **********************") 
            print(DrivetrainGenerator.instance.getName())
        return DrivetrainGenerator.instance
