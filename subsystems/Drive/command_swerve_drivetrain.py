from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
import math
from phoenix6 import SignalLogger, swerve, units, utils, hardware
from phoenix6.swerve.requests import ForwardPerspectiveValue
from phoenix6.configs import Slot0Configs, Slot1Configs
from typing import Callable, overload
from wpilib import DriverStation, Notifier, RobotController, SmartDashboard
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Pose2d, Rotation2d
from Constants1 import ConstantValues


class CommandSwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain[hardware.TalonFX, hardware.TalonFX, hardware.CANcoder]):

    """
    Class that extends the Phoenix 6 SwerveDrivetrain class and implements
    Subsystem so it can easily be used in command-based projects.
    """

    _SIM_LOOP_PERIOD: units.second = 0.005

    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    """Blue alliance sees forward as 0 degrees (toward red alliance wall)"""
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    """Red alliance sees forward as 180 degrees (toward blue alliance wall)"""

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:     Type of the drive motor
        :type drive_motor_type:      type
        :param steer_motor_type:     Type of the steer motor
        :type steer_motor_type:      type
        :param encoder_type:         Type of the azimuth encoder
        :type encoder_type:          type
        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:  swerve.SwerveDrivetrainConstants
        :param modules:              Constants for each specific module
        :type modules:               list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
#        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:            Type of the drive motor
        :type drive_motor_type:             type
        :param steer_motor_type:            Type of the steer motor
        :type steer_motor_type:             type
        :param encoder_type:                Type of the azimuth encoder
        :type encoder_type:                 type
        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
#        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        odometry_standard_deviation: tuple[float, float, float],
        vision_standard_deviation: tuple[float, float, float],
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:            Type of the drive motor
        :type drive_motor_type:             type
        :param steer_motor_type:            Type of the steer motor
        :type steer_motor_type:             type
        :param encoder_type:                Type of the azimuth encoder
        :type encoder_type:                 type
        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param odometry_standard_deviation: The standard deviation for odometry calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type odometry_standard_deviation:  tuple[float, float, float]
        :param vision_standard_deviation:   The standard deviation for vision calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type vision_standard_deviation:    tuple[float, float, float]
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        arg0=None,
        arg1=None,
        arg2=None,
        arg3=None,
    ):
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(
            self, drive_motor_type, steer_motor_type, encoder_type,
            drivetrain_constants, arg0, arg1, arg2, arg3
        )
        self.rot_deg=0 
        self.reset_pose(Pose2d())
        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        self._has_applied_operator_perspective = False
        """Keep track if we've ever applied the operator perspective before or not"""
        self.setup_swerve_requests()
        # Swerve requests to apply during SysId characterization
        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotation_characterization = swerve.requests.SysIdSwerveRotation()

        self._sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing translation. This is used to find PID gains for the drive motors."""

        self._sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._steer_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing steer. This is used to find PID gains for the steer motors."""

        self._sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate=math.pi / 6,
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Use default timeout (10 s)
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: (
                    # output is actually radians per second, but SysId only supports "volts"
                    self.set_control(
                        self._rotation_characterization.with_rotational_rate(output)
                    ),
                    # also log the requested output for SysId
                    SignalLogger.write_double("Rotational_Rate", output),
                ),
                lambda log: None,
                self,
            ),
        )
        """
        SysId routine for characterizing rotation.
        This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
        See the documentation of swerve.requests.SysIdSwerveRotation for info on importing the log to SysId.
        """

        self._sys_id_routine_to_apply = self._sys_id_routine_translation
        """The SysId routine to test"""
        if utils.is_simulation():
            self._start_sim_thread()
    def apply_request(
        self, request: Callable[[], swerve.requests.SwerveRequest]
    ) -> Command:
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Quasistatic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Dynamic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.dynamic(direction)

    def periodic(self):
        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.nc
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True


     
       
#        self.operator_fwd_dir_deg = self.get_operator_forward_direction().degrees()    
       # pose =  self.get_pose()
       # spd = self.get_speeds()
        #SmartDashboard.putNumber("Vx: ",round(spd.vx,3))
        #SmartDashboard.putNumber("Vy: ",round(spd.vy,3))
        #SmartDashboard.putNumber("Rot Rate: ",round(self.get_omega_dps(),3))   
        #SmartDashboard.putNumber("X: ",round(pose.X(),3))
        #SmartDashboard.putNumber("y: ",round(pose.Y(),3))
        #SmartDashboard.putNumber("Rot: ",round(self.get_rotation_deg(),3))  


    def _start_sim_thread(self):
        def _sim_periodic():
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)

    def add_vision_measurement(self, vision_robot_pose: Pose2d, timestamp: units.second, vision_measurement_std_devs: tuple[float, float, float] | None = None):
        """
        Adds a vision measurement to the Kalman Filter. This will correct the
        odometry pose estimate while still accounting for measurement noise.

        Note that the vision measurement standard deviations passed into this method
        will continue to apply to future measurements until a subsequent call to
        set_vision_measurement_std_devs or this method.

        :param vision_robot_pose:           The pose of the robot as measured by the vision camera.
        :type vision_robot_pose:            Pose2d
        :param timestamp:                   The timestamp of the vision measurement in seconds.
        :type timestamp:                    second
        :param vision_measurement_std_devs: Standard deviations of the vision pose measurement
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians.
        :type vision_measurement_std_devs:  tuple[float, float, float] | None
        """
        swerve.SwerveDrivetrain.add_vision_measurement(self, vision_robot_pose, utils.fpga_to_current_time(timestamp), vision_measurement_std_devs)
               
    

    def setup_swerve_requests(self):
        self.request_teleop_FC = (
            swerve.requests.FieldCentric()
            .with_deadband(ConstantValues.DriveConstants.SPEED_AT_12_VOLTS *
                    ConstantValues.DriveConstants.TELEOP_DEADBAND)  #squared input, so db starts at 0.05
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY)
        )
        
        self.request_teleop_FC_facing = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(ConstantValues.DriveConstants.SPEED_AT_12_VOLTS *
                    ConstantValues.DriveConstants.TELEOP_DEADBAND)  #squared input, so db starts at 0.05
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_heading_pid(
                ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KP,
                0,
                ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KD)    
        )


        self.request_autopilot = (
            swerve.requests.FieldCentricFacingAngle()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_forward_perspective(ForwardPerspectiveValue.BLUE_ALLIANCE)                
            .with_heading_pid(
                ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KP,
                0,
                ConstantValues.HeadingControllerConstants.HEADINGCONTROLLER_KD)    
        )        

        self.request_RC = (swerve.requests.RobotCentric().with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY))


    def drive_FC(self,x_vel,y_vel,rot_vel):
        self.set_control(
                self.request_teleop_FC.
                with_velocity_x(x_vel).
                with_velocity_y(y_vel).
                with_rotational_rate(rot_vel))

        
    def drive_FC_facing(self,x_vel,y_vel,angle):
        self.set_control(
                self.request_teleop_FC_facing
                .with_velocity_x(x_vel)
                .with_velocity_y(y_vel)
                .with_target_direction(Rotation2d(angle)))
        
    def drive_autopilot(self,x_vel,y_vel,angle):
        self.set_control(self.request_autopilot
                .with_velocity_x(x_vel)
                .with_velocity_y(y_vel)
                .with_target_direction(Rotation2d(angle)))
        

    def drive_RC(self,x_vel,y_vel,rot_vel):
        self.set_control(
                self.request_RC
                .with_velocity_x(x_vel)
                .with_velocity_y(y_vel)
                .with_rotational_rate(rot_vel)
                )
    
    def get_speeds(self):
        return self.get_state().speeds
    
    def get_pose(self):
        return self.get_state().pose
    
    def get_speeds_norm(self):
        speed = self.get_state().speeds
        return  math.hypot(speed.vx,speed.vy)       

    def get_X(self):
        return  self.get_state().pose.X()

    def get_Y(self):
        return  self.get_state().pose.Y()                  

    def get_rotation_deg(self):
        return  self.get_state().pose.rotation().degrees()

    def get_rotation_rad(self):
        return  self.get_state().pose.rotation().radians()   

    def get_omega_rps(self):
        return self.get_state().speeds.omega
    
    def get_omega_dps(self):
        return self.get_state().speeds.omega_dps                   

    
    def update(self):
        slot1_auto = Slot1Configs()
        slot0_teleop = Slot0Configs()

        k_p_tele = ConstantValues.DriveConstants.TELEOP_kP
        k_v_tele = ConstantValues.DriveConstants.TELEOP_kV
        k_s_tele = ConstantValues.DriveConstants.TELEOP_kS
        k_a_tele = ConstantValues.DriveConstants.TELEOP_kA
        
        slot0_teleop.k_p = k_p_tele
        slot0_teleop.k_s = k_s_tele
        slot0_teleop.k_v = k_v_tele
        slot0_teleop.k_a = k_a_tele                        

        k_p_auto = ConstantValues.DriveConstants.AUTO_kP
        k_v_auto = ConstantValues.DriveConstants.AUTO_kV
        k_s_auto = ConstantValues.DriveConstants.AUTO_kS
        k_a_auto = ConstantValues.DriveConstants.AUTO_kA

        slot1_auto.k_p = k_p_auto
        slot1_auto.k_s = k_s_auto
        slot1_auto.k_v = k_v_auto
        slot1_auto.k_a = k_a_auto                                


        self.get_module(0).drive_motor.configurator.apply(slot0_teleop)
        self.get_module(1).drive_motor.configurator.apply(slot0_teleop)
        self.get_module(2).drive_motor.configurator.apply(slot0_teleop)
        self.get_module(3).drive_motor.configurator.apply(slot0_teleop)       


        self.get_module(0).drive_motor.configurator.apply(slot1_auto)
        self.get_module(1).drive_motor.configurator.apply(slot1_auto)
        self.get_module(2).drive_motor.configurator.apply(slot1_auto)
        self.get_module(3).drive_motor.configurator.apply(slot1_auto)        
        
        self.setup_swerve_requests()


