import commands2
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator
from phoenix6 import swerve
from subsystems.Drive.heading_controller import HeadingController
from  subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain

class StopDrive(commands2.Command):
    def __init__(self):
        self.headingController = HeadingController.getInstance() 
        self.drivetrain = DrivetrainGenerator.getInstance()

    def execute(self) -> None:
        self.headingController.setTargetRotationInt(True)
        self.drivetrain.drive_FC(0,0,0)

        
    def isFinished(self):
        return True