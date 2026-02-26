import commands2
from subsystems.Drive.heading_controller import HeadingController
from  subsystems.Drive.command_swerve_drivetrain import CommandSwerveDrivetrain



class SeedZero(commands2.Command):    
    def __init__(
        self,
        _drivetrain: CommandSwerveDrivetrain, 
        _headingController: HeadingController
        ) -> None:
        
        self.drivetrain = _drivetrain
        self.headingController = _headingController
        self.addRequirements(self.drivetrain)


    def execute(self) -> None:
        self.drivetrain.drive_RC(0,0,0)
        self.headingController.zeroStateAngle()
        self.drivetrain.seed_field_centric()
        

    def end(self, interrupted: bool) :
        self.drivetrain.drive_RC(0,0,0)

    def isFinished(self) -> bool:
        return True