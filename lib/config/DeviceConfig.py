from phoenix6.configs import (
    CANcoderConfiguration,
    ClosedLoopGeneralConfigs,
    Slot0Configs,
    ClosedLoopRampsConfigs,
    CurrentLimitsConfigs,
    FeedbackConfigs,
    MotorOutputConfigs,
    OpenLoopRampsConfigs,
    Pigeon2Configuration,
    TalonFXConfiguration,
    TorqueCurrentConfigs,
)
from phoenix6.hardware import CANcoder, Pigeon2, TalonFX
from phoenix6.signals import (
    AbsoluteSensorRangeValue,
    FeedbackSensorSourceValue,
    InvertedValue,
    NeutralModeValue,
)

from wpimath.geometry import Rotation2d

from lib.config.ErrorChecker import DeviceConfiguration, ErrorChecker
from lib.pid.ScreamPIDConstants import ScreamPIDConstants

from constants import SwerveConstants, SteerConstants, DriveConstants


class DeviceConfig:
    @staticmethod
    def driveFXConfig():
        config = TalonFXConfiguration()
        config.audio.beep_on_boot = False
        config.audio.beep_on_config = False
        config.motor_output = DeviceConfig.FXMotorOutputConfig(
            DriveConstants.MOTOR_INVERT, DriveConstants.NEUTRAL_MODE
        )
        config.feedback = DeviceConfig.FXFeedbackConfig(
            FeedbackSensorSourceValue.ROTOR_SENSOR,
            0,
            DriveConstants.GEAR_RATIO,
            Rotation2d.fromRotations(0),
        )
        config.current_limits = DeviceConfig.FXCurrentLimitsConfig(
            DriveConstants.CURRENT_LIMIT_ENABLE,
            DriveConstants.SUPPLY_CURRENT_LIMIT,
            DriveConstants.SUPPLY_CURRENT_THRESHOLD,
            DriveConstants.SUPPLY_TIME_THRESHOLD,
        )
        config.slot0 = DeviceConfig.FXPIDConfig(DriveConstants.PID_CONSTANTS)
        config.open_loop_ramps = DeviceConfig.FXOpenLoopRampConfig(
            DriveConstants.OPEN_LOOP_RAMP
        )
        config.closed_loop_ramps = DeviceConfig.FXClosedLoopRampConfig(
            DriveConstants.CLOSED_LOOP_RAMP
        )
        config.torque_current = DeviceConfig.FXTorqueCurrentConfig(
            DriveConstants.SLIP_CURRENT, -DriveConstants.SLIP_CURRENT, 0
        )
        return config

    @staticmethod
    def steerFXConfig(remoteSensorID: int):
        config = TalonFXConfiguration()
        config.audio.beep_on_boot = False
        config.audio.beep_on_config = False
        config.motor_output = DeviceConfig.FXMotorOutputConfig(
            SteerConstants.MOTOR_INVERT, SteerConstants.NEUTRAL_MODE
        )
        config.feedback = DeviceConfig.FXSteerFeedbackConfig(
            FeedbackSensorSourceValue.FUSED_CANCODER,
            remoteSensorID,
            SteerConstants.GEAR_RATIO,
            Rotation2d.fromRotations(0),
        )
        config.closed_loop_general = DeviceConfig.FXClosedLoopGeneralConfig(True)
        config.current_limits = DeviceConfig.FXCurrentLimitsConfig(
            SteerConstants.CURRENT_LIMIT_ENABLE,
            SteerConstants.SUPPLY_CURRENT_LIMIT,
            SteerConstants.SUPPLY_CURRENT_THRESHOLD,
            SteerConstants.SUPPLY_TIME_THRESHOLD,
        )
        config.slot0 = DeviceConfig.FXPIDConfig(SteerConstants.PID_CONSTANTS)
        return config

    @staticmethod
    def swerveEncoderConfig(angleOffset: Rotation2d):
        config = CANcoderConfiguration()
        config.magnet_sensor.absolute_sensor_range = (
            AbsoluteSensorRangeValue.UNSIGNED_0_TO1
        )
        config.magnet_sensor.sensor_direction = (
            SwerveConstants.MODULE_TYPE.CANcoderInvert
        )
        config.magnet_sensor.magnet_offset = angleOffset.degrees()
        return config

    @staticmethod
    def swervePigeonConfig():
        config = Pigeon2Configuration()
        return config

    @staticmethod
    def configureTalonFX(
        name: str,
        motor: TalonFX,
        config: TalonFXConfiguration,
        update_frequency_hz: float,
    ):
        deviceConfig = DeviceConfiguration()
        deviceConfig.configureSettings = (
            lambda: ErrorChecker.hasConfiguredWithoutErrors(
                motor.configurator.apply(config),
                motor.configurator.set_position(0),
                motor.get_duty_cycle().set_update_frequency(update_frequency_hz),
                motor.get_position().set_update_frequency(update_frequency_hz),
                motor.get_velocity().set_update_frequency(update_frequency_hz),
                motor.optimize_bus_utilization(),
            )
        )
        ErrorChecker.configureDevice(
            deviceConfig,
            f"{name} {motor.device_id} version {motor.get_version().value}",
            True,
        )

    @staticmethod
    def configureSwerveEncoder(
        name: str,
        encoder: CANcoder,
        config: CANcoderConfiguration,
        updateFrequencyHz: float,
    ):
        deviceConfig = DeviceConfiguration()
        deviceConfig.configureSettings = (
            lambda: ErrorChecker.hasConfiguredWithoutErrors(
                encoder.configurator.apply(config),
                encoder.get_position().set_update_frequency(updateFrequencyHz),
                encoder.get_absolute_position().set_update_frequency(updateFrequencyHz),
                encoder.optimize_bus_utilization(),
            )
        )
        ErrorChecker.configureDevice(
            deviceConfig,
            f"{name} {encoder.device_id} version {encoder.get_version().value}",
            True,
        )

    @staticmethod
    def configurePigeon2(
        name: str,
        pigeon: Pigeon2,
        config: Pigeon2Configuration,
        updateFrequencyHz: float,
    ):
        deviceConfig = DeviceConfiguration()
        deviceConfig.configureSettings = (
            lambda: ErrorChecker.hasConfiguredWithoutErrors(
                pigeon.configurator.apply(config),
                pigeon.set_yaw(0),
                pigeon.get_yaw().set_update_frequency(updateFrequencyHz),
                pigeon.get_pitch().set_update_frequency(updateFrequencyHz),
                pigeon.get_roll().set_update_frequency(updateFrequencyHz),
                pigeon.optimize_bus_utilization(),
            )
        )
        ErrorChecker.configureDevice(
            deviceConfig,
            f"{name} {pigeon.device_id} version {pigeon.get_version().value}",
            True,
        )

    @staticmethod
    def FXMotorOutputConfig(inverted: InvertedValue, neutral_mode: NeutralModeValue):
        config = MotorOutputConfigs()
        config.inverted = inverted
        config.neutral_mode = neutral_mode
        return config

    @staticmethod
    def FXFeedbackConfig(
        sensor: FeedbackSensorSourceValue,
        remoteSensorID: int,
        sensorToMechGR: float,
        sensorOffset: Rotation2d,
    ):
        config = FeedbackConfigs()
        config.feedback_sensor_source = sensor
        config.feedback_remote_sensor_id = remoteSensorID
        config.sensor_to_mechanism_ratio = sensorToMechGR
        config.feedback_rotor_offset = sensorOffset.degrees() / 360
        return config

    @staticmethod
    def FXSteerFeedbackConfig(
        sensor: FeedbackSensorSourceValue,
        remote_sensor_id: int,
        rotor_to_sensor_gr: float,
        sensor_offset: Rotation2d,
    ):
        config = FeedbackConfigs()
        config.feedback_sensor_source = sensor
        config.feedback_remote_sensor_id = remote_sensor_id
        config.rotor_to_sensor_ratio = rotor_to_sensor_gr
        config.feedback_rotor_offset = sensor_offset.degrees() / 360
        return config

    @staticmethod
    def FXPIDConfig(pid: ScreamPIDConstants):
        return pid.toSlot0Configs()

    @staticmethod
    def FXOpenLoopRampConfig(ramp: float):
        config = OpenLoopRampsConfigs()
        config.duty_cycle_open_loop_ramp_period = ramp
        return config

    @staticmethod
    def FXClosedLoopRampConfig(ramp: float):
        config = ClosedLoopRampsConfigs()
        config.duty_cycle_closed_loop_ramp_period = ramp
        return config

    @staticmethod
    def FXCurrentLimitsConfig(
        enable: bool, limit: float, currentThreshold: float, timeThreshold: float
    ):
        config = CurrentLimitsConfigs()
        config.supply_current_limit_enable = enable
        config.supply_current_limit = limit
        config.supply_current_threshold = currentThreshold
        config.supply_time_threshold = timeThreshold
        return config

    @staticmethod
    def FXClosedLoopGeneralConfig(continuousWrap: bool):
        config = ClosedLoopGeneralConfigs()
        config.continuous_wrap = continuousWrap
        return config

    @staticmethod
    def FXTorqueCurrentConfig(
        peakForward: float, peakReverse: float, neutralDeadband: float
    ):
        config = TorqueCurrentConfigs()
        config.peak_forward_torque_current = peakForward
        config.peak_reverse_torque_current = peakReverse
        config.torque_neutral_deadband = neutralDeadband
        return config
