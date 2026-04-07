from commands2 import Command, InstantCommand, Subsystem
from wpilib import Timer
from math import pi
from Constants1 import ConstantValues
from subsystems.Drive.drivetrain_generator import DrivetrainGenerator 

#To Do - check if rotation not being controlled until first rotation

class HeadingController(Subsystem):
    instance=None
    
    @staticmethod
    def getInstance():
        if HeadingController.instance == None:
            HeadingController.instance = HeadingController()
            print("**********************  HEADING CONTROLLER  **********************") 
        return HeadingController.instance
    
    def __init__(self):
        self.timer = Timer()
        self.timer.reset()
        self.timer.start()
        self.time1 = 0 
        #  state values: 0 = rotating by stick, 1 = heading hold, 2 = snap/hold complete, 3 = forced snap direction               
        self.state =0 

        self.driveTrain = DrivetrainGenerator.getInstance()
        
        self.rotation = 0.0
        self.targetRotation = 0

        self.snap_rotation_direction = 0
        self.snap_tolerance = ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_TOLERANCE
        self.snap_away_from_wall_distance = ConstantValues.HeadingControllerConstants.SNAP_AWAY_FROM_WALL_DISTANCE
        self.snap_wall_swing_clearance = ConstantValues.HeadingControllerConstants.SNAP_WALL_SWING_CLEARANCE
        self.snap_forced_kp = ConstantValues.HeadingControllerConstants.SNAP_FORCED_KP
        self.snap_forced_min_rate = ConstantValues.HeadingControllerConstants.SNAP_FORCED_MIN_RATE
        self.last_rotation_wrapped = self.driveTrain.rotation_rad-self.driveTrain.get_operator_forward_direction().radians()
        self.rotation_unwrapped = 0.0
        self.snap_target_unwrapped = None


    def get_rotation_state(self,stick_rot):
        prev_state = self.state
        self.rotation = self.driveTrain.rotation_rad-self.driveTrain.get_operator_forward_direction().radians()
        
        delta_rotation = self.wrap_to_pi(self.rotation - self.last_rotation_wrapped)
        self.rotation_unwrapped += delta_rotation
        self.last_rotation_wrapped = self.rotation

        if self.state == 3:
            error = self.snap_target_unwrapped - self.rotation_unwrapped
            if abs(error) <= self.snap_tolerance:
                self.state = 2
                self.snap_rotation_direction = 0
                self.snap_target_unwrapped = None
            else:
                self.snap_rotation_direction = 1 if error > 0 else -1
        
        if (abs(stick_rot) > 0):
            self.targetRotation = self.rotation
            self.state=0
            self.snap_rotation_direction = 0
            self.snap_target_unwrapped = None

        elif self.state not in (2,3):  
            self.state = 1
            # Capture heading when transitioning from manual rotation to hold mode.
            # This also handles first enable when no rotation has happened yet.
            if prev_state == 0 or self.timer.get() - self.time1 < 1:
                self.targetRotation = self.rotation
        return self.state, self.targetRotation


    def rotateToZero(self):
        if self.is_near_wall():
            self.start_snap_away_from_wall(0)
        else:
            self.setTargetRotation(0)
            self.state=2


    def rotateTo90(self):
        self.setTargetRotation(pi/2)    
        self.state=2


    def rotateTo180(self):
        if self.is_near_wall():
            self.start_snap_away_from_wall(pi)
        else:
            self.setTargetRotation(pi)
            self.state=2


    def rotateTo270(self):
        self.setTargetRotation(3*pi/2)
        self.state=2        


    def rotateToZeroCommand(self):
        return self.runOnce(lambda: self.rotateToZero())
        
    
    def rotateTo90Command(self):
        return self.runOnce(lambda: self.rotateTo90())

    def rotateTo180Command(self):
        return self.runOnce(lambda: self.rotateTo180())

    def rotateTo270Command(self):
        return self.runOnce(lambda: self.rotateTo270())

    def set_forward_direction(self):
        self.driveTrain.drive_RC(0,0,0)
        self.state=0
        self.driveTrain.set_operator_perspective_forward(
                self.driveTrain.pose.rotation())
                

    def set_forward_directionCommand(self):
        return self.runOnce(self.set_forward_direction())

    def setTargetRotation(self,angle : float):
        self.targetRotation = angle



    def get_snap_rotation_rate(self, max_angular_rate: float) -> float:
        if self.state != 3:
            return 0.
        
#        if self.snap_target_unwrapped is not None:
#            error = self.snap_target_unwrapped - self.rotation_unwrapped
#            if abs(error) <= self.snap_tolerance:
#                return 0.0
        error = self.snap_target_unwrapped - self.rotation_unwrapped
        abs_error = abs(error)
        if abs_error <= self.snap_tolerance:
            return 0.0
        commanded_rate = self.snap_forced_kp * abs_error
        commanded_rate = min(commanded_rate, max_angular_rate)
        commanded_rate = max(commanded_rate, self.snap_forced_min_rate)
        self.snap_rotation_direction = 1 if error > 0 else -1
        return self.snap_rotation_direction * commanded_rate

    def start_snap_away_from_wall(self, target_angle: float):
        self.rotation = self.driveTrain.rotation_rad-self.driveTrain.get_operator_forward_direction().radians()
        delta_rotation = self.wrap_to_pi(self.rotation - self.last_rotation_wrapped)
        self.rotation_unwrapped += delta_rotation
        self.last_rotation_wrapped = self.rotation
        self.targetRotation = target_angle
        short_delta_to_target = self.wrap_to_pi(target_angle - self.rotation)
        if abs(short_delta_to_target) <= self.snap_tolerance:
            self.state = 2
            self.snap_rotation_direction = 0
            self.snap_target_unwrapped = None
            return

        if not self.is_snap_toward_wall_risky(short_delta_to_target):
            self.state = 2
            self.snap_rotation_direction = 0
            self.snap_target_unwrapped = None
            return

        away_heading = self.get_away_heading_relative_to_operator()
        error_to_away = self.wrap_to_pi(away_heading - self.rotation)
        preferred_sign = 1 if error_to_away >= 0 else -1
        delta_to_target = short_delta_to_target
        if (1 if delta_to_target > 0 else -1) != preferred_sign:
            delta_to_target += preferred_sign * 2 * pi

        self.snap_target_unwrapped = self.rotation_unwrapped + delta_to_target
        self.snap_rotation_direction = preferred_sign
        self.state = 3

    def wrap_to_pi(self, angle: float) -> float:
        while angle > pi:
            angle -= 2 * pi
        while angle <= -pi:
            angle += 2 * pi
        return angle

    def is_near_wall(self) -> bool:
        pose = self.driveTrain.pose
        x = pose.X()
        y = pose.Y()
        field_length = 16.541
        field_width = 8.069
        nearest_wall_distance = min(x, field_length - x, y, field_width - y)
        return nearest_wall_distance <= self.snap_away_from_wall_distance

    def get_away_heading_relative_to_operator(self) -> float:
        pose = self.driveTrain.pose
        x = pose.X()
        y = pose.Y()
        field_length = 16.541
        field_width = 8.069

        wall_distances = {
            "x_min": x,
            "x_max": field_length - x,
            "y_min": y,
            "y_max": field_width - y,
        }
        nearest_wall = min(wall_distances, key=wall_distances.get)
        away_heading_field = 0.0
        if nearest_wall == "x_max":
            away_heading_field = pi
        elif nearest_wall == "y_min":
            away_heading_field = pi / 2
        elif nearest_wall == "y_max":
            away_heading_field = -pi / 2

        operator_forward = self.driveTrain.get_operator_forward_direction().radians()
        return self.wrap_to_pi(away_heading_field - operator_forward)

    def get_nearest_wall(self) -> str:
        pose = self.driveTrain.pose
        x = pose.X()
        y = pose.Y()
        field_length = 16.541
        field_width = 8.069

        wall_distances = {
            "x_min": x,
            "x_max": field_length - x,
            "y_min": y,
            "y_max": field_width - y,
        }
        return min(wall_distances, key=wall_distances.get)

    def get_toward_wall_heading_relative_to_operator(self) -> float:
        nearest_wall = self.get_nearest_wall()
        toward_heading_field = pi
        if nearest_wall == "x_max":
            toward_heading_field = 0.0
        elif nearest_wall == "y_min":
            toward_heading_field = -pi / 2
        elif nearest_wall == "y_max":
            toward_heading_field = pi / 2

        operator_forward = self.driveTrain.get_operator_forward_direction().radians()
        return self.wrap_to_pi(toward_heading_field - operator_forward)

    def is_snap_toward_wall_risky(self, delta_to_target: float) -> bool:
        pose = self.driveTrain.pose
        x = pose.X()
        y = pose.Y()
        field_length = 16.541
        field_width = 8.069
        nearest_wall_distance = min(x, field_length - x, y, field_width - y)

        if nearest_wall_distance > self.snap_wall_swing_clearance:
            return False

        toward_wall_heading = self.get_toward_wall_heading_relative_to_operator()
        toward_wall_unwrapped = self.rotation_unwrapped + self.wrap_to_pi(toward_wall_heading - self.rotation)
        path_start = self.rotation_unwrapped
        path_end = self.rotation_unwrapped + delta_to_target
        lower_bound = min(path_start, path_end)
        upper_bound = max(path_start, path_end)

        for wrap_count in (-1, 0, 1):
            candidate = toward_wall_unwrapped + wrap_count * 2 * pi
            if lower_bound <= candidate <= upper_bound:
                return True
        return False

#  Returns a command for setting target rotation, for use in auto command sequences and button bindings
    def setTargetRotationCommand(self,angle) -> Command:
        return  InstantCommand(lambda: self.setTargetRotation(angle))   
    
    
# Set target rotation, used for finally_do decorators for commands  that require a supplied boolean.
# The boolean indicates if the command was interrupted or ended naturally
    def setTargetRotationInt (self,b:bool):
        self.state=2
        self.setTargetRotation(self.driveTrain.rotation_rad-self.driveTrain.get_operator_forward_direction().radians())

#  Returns a command for setting target rotation, for use in auto command sequences and button bindings
    def setTargetRotationToSelfCommand(self) -> Command:
        return  InstantCommand(lambda: self.setTargetRotationInt(True))   
    
    
