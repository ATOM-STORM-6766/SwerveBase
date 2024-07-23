from phoenix6 import BaseStatusSignal, StatusSignal
from phoenix6.controls import (
    PositionVoltage,
    VelocityTorqueCurrentFOC,
    VoltageOut,
)
from phoenix6.configs import MotorOutputConfigs
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.signals import NeutralModeValue

from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from lib.config.DeviceConfig import DeviceConfig
from lib.math.Conversions import Conversions
from lib.pid.ScreamPIDConstants import ScreamPIDConstants

import constants
from constants import Ports
from constants import SwerveConstants
from constants import DriveConstants
from constants import ModuleConstants


class SwerveModule:
    """
    A swerve module, which consists of an angle motor, a drive motor, and an angle encoder.

    Provides methods to get and set all parts of the module, such as the speed and angle.
    """

    m_modNumber: int
    m_modLocation: str
    m_angleOffset: Rotation2d
    m_lastAngle: Rotation2d

    m_steerMotor: TalonFX
    m_driveMotor: TalonFX
    m_angleEncoder: CANcoder

    m_signals: list[BaseStatusSignal]
    m_drivePosition: StatusSignal[float]
    m_driveVelocity: StatusSignal[float]
    m_steerPosition: StatusSignal[float]
    m_steerVelocity: StatusSignal[float]
    m_internalState = SwerveModulePosition()

    m_voltage_out = VoltageOut(0)
    m_velocity_torque_current_FOC = VelocityTorqueCurrentFOC(0)
    m_position_voltage = PositionVoltage(0)

    m_feedforward = SimpleMotorFeedforwardMeters(
        DriveConstants.KS, DriveConstants.KV, DriveConstants.KA
    )

    def __init__(
        self,
        module: ModuleConstants.ModuleLocation,
        constants: ModuleConstants.SwerveModuleConstants,
    ):
        """
        Constructs a `SwerveModule` object with the given location and module constants.

        param:
        - `moduleLocation`: the `ModuleLocation` to get the location and number from.
        - `constants`: The constants to use for the module.Set each module's constants in `ModuleConstants`.
        """
        self.m_modNumber = module.getNumber()
        self.m_modLocation = module.toString()
        self.m_angleOffset = constants.angleOffset

        # Angle Encoder Config
        self.m_angleEncoder = CANcoder(constants.encoderID, Ports.CAN_BUS_NAME)
        self.configAngleEncoder()

        self.m_steerMotor = TalonFX(constants.steerMotorID, Ports.CAN_BUS_NAME)
        self.configSteerMotor()

        self.m_driveMotor = TalonFX(constants.driveMotorID, Ports.CAN_BUS_NAME)
        self.configDriveMotor()

        self.m_drivePosition = self.m_driveMotor.get_position()
        self.m_driveVelocity = self.m_driveMotor.get_velocity()
        self.m_steerPosition = self.m_steerMotor.get_position()
        self.m_steerVelocity = self.m_steerMotor.get_velocity()

        self.m_signals = [
            self.m_drivePosition,
            self.m_driveVelocity,
            self.m_steerPosition,
            self.m_steerVelocity,
        ]

        self.m_lastAngle = self.getState(True).angle

    def getLocation(self) -> str:
        """
        Returns the module location associated with this module.

        Returns:

        - The module location.
        """
        return self.m_modLocation

    def getModuleNumber(self) -> int:
        """
        Returns the module number associated with this module.

        Returns:

        - The module number.
        """
        return self.m_modNumber

    def setSteerNeutralMode(self, mode: NeutralModeValue):
        """
        Sets the neutral mode of the steer motor.

        Use `NeutralModeValue.Brake` or `NeutralModeValue.Coast`.

        Parameters:

        - `mode`: The `NeutralModeValue` to set.
        """
        cfg = MotorOutputConfigs()
        cfg.neutral_mode = mode
        cfg.inverted = DriveConstants.MOTOR_INVERT
        self.m_steerMotor.configurator.refresh(cfg)

    def setDriveNeutralMode(self, mode: NeutralModeValue):
        """
        Sets the neutral mode of the drive motor.

        Use `NeutralModeValue.Brake` or `NeutralModeValue.Coast`.

        Parameters:

        - `mode`: The `NeutralModeValue` to set.
        """
        cfg = MotorOutputConfigs()
        cfg.neutral_mode = mode
        cfg.inverted = DriveConstants.MOTOR_INVERT
        self.m_steerMotor.configurator.refresh(cfg)

    def set(self, desiredState: SwerveModuleState, isOpenLoop: bool):
        """
        Sets the desired state of the Swerve module, including the speed and angle.

        Parameters:
        - `desiredState`: The desired state of the Swerve module.
        - `isOpenLoop`: Whether the desired state is open loop (Tele-Op driving), or closed loop (Autonomous driving).
        """
        desiredState = SwerveModuleState.optimize(desiredState, self.getAngle())
        self.setSpeed(desiredState, isOpenLoop)
        self.setAngle(desiredState)

    def setSpeed(self, desiredState: SwerveModuleState, isOpenLoop: bool):
        """
        sets the speed of the swerve module based on the desired state's speed and whether it is in open loop or closed loop control.

        Parameters:
        - `desiredState`: The desired state of the swerve module.
        - `isOpenLoop`: Whether to drive in an open loop (Tele-Op) or closed loop (Autonomous) state.
        """
        if isOpenLoop:
            self.m_driveMotor.set_control(
                self.m_voltage_out.with_output(
                    (desiredState.speed / SwerveConstants.MAX_SPEED) * 12
                ).with_enable_foc(True)
            )
        else:
            velocity = Conversions.mpsToFalconRPS(
                desiredState.speed,
                SwerveConstants.MODULE_TYPE.wheelCircumference,
                1,
            )
            feedforward = self.m_feedforward.calculate(desiredState.speed)
            self.m_driveMotor.set_control(
                self.m_velocity_torque_current_FOC.with_velocity(
                    velocity
                ).with_feed_forward(feedforward)
            )

    def setAngle(self, desiredState: SwerveModuleState):
        """
        Sets the angle of the swerve module based on the desired state's angle.

        Parameters:
        - `desiredState`: The desired state of the swerve module.
        """

        # Prevent rotating module if speed is less then 1%. Prevents jittering when not moving.
        angle = (
            self.m_lastAngle
            if abs(desiredState.speed) <= (SwerveConstants.MAX_SPEED * 0.01)
            else desiredState.angle
        )
        self.m_steerMotor.set_control(
            self.m_position_voltage.with_position(angle.degrees() / 360)
        )
        self.m_lastAngle = angle

    def getAngle(self) -> Rotation2d:
        """
        Retrieves the current angle of the steer motor.
        Returns:
        - The angle of the steer motor as a Rotation2d.
        """
        return self.m_internalState.angle

    def getEncoderAngle(self) -> Rotation2d:
        """
        Retrieves the current angle of the steer motor encoder.
        Returns:
        - The angle of the steer motor encoder as a Rotation2d.
        """
        return Rotation2d.fromRotations(
            self.m_angleEncoder.get_absolute_position().value
        )

    def configAngleEncoder(self):
        """
        Configures the angle encoder.
        See DeviceConfig for more information.
        """
        DeviceConfig.configureSwerveEncoder(
            self.m_modLocation + " Angle Encoder",
            self.m_angleEncoder,
            DeviceConfig.swerveEncoderConfig(self.m_angleOffset),
            constants.LOOP_TIME_HZ,
        )

    def configSteerMotor(self):
        """
        Configures the steer motor.
        See DeviceConfig for more information.
        """
        DeviceConfig.configureTalonFX(
            self.m_modLocation + " Steer Motor",
            self.m_steerMotor,
            DeviceConfig.steerFXConfig(self.m_angleEncoder.device_id),
            constants.LOOP_TIME_HZ,
        )

    def configDriveMotor(self):
        """
        Configures the drive motor.
        See DeviceConfig for more information.
        """
        DeviceConfig.configureTalonFX(
            self.m_modLocation + " Drive Motor",
            self.m_driveMotor,
            DeviceConfig.driveFXConfig(),
            constants.LOOP_TIME_HZ,
        )

    def configDriveMotorPID(self, constants: ScreamPIDConstants):
        """Configures the drive motor with the given constants.

        Parameters:

        - `constants`: The `ScreamPIDConstants` to be applied."""
        self.m_driveMotor.configurator.apply(constants.toSlot0Configs())

    def getState(self, refresh: bool) -> SwerveModulePosition:
        """
        retrieves the current state of the swerve module.

        Parameters:

        - `refresh`: Whether to refresh the readings from the motors.

        Returns:

        - The state of the swerve module, including the velocity (m/s) and angle.
        """
        if refresh:
            self.m_driveVelocity.refresh()
            self.m_steerPosition.refresh()

        speedMetersPerSecond = Conversions.falconRPSToMechanismMPS(
            self.m_driveVelocity.value,
            SwerveConstants.MODULE_TYPE.wheelCircumference,
            1,
        )
        angle = Rotation2d.fromDegrees(self.m_steerPosition.value)
        return SwerveModuleState(speedMetersPerSecond, angle)

    def getPosition(self, refresh: bool) -> SwerveModulePosition:
        """
        Retrieves the current position of the swerve module.

        Parameters:
        - `refresh`: Whether to refresh the readings from the motors.

        Returns:
        - The position of the swerve module, including the velocity (m/s) and angle.
        """
        if refresh:
            self.m_drivePosition.refresh()
            self.m_driveVelocity.refresh()
            self.m_steerPosition.refresh()
            self.m_steerVelocity.refresh()

        driveRotations = BaseStatusSignal.get_latency_compensated_value(
            self.m_drivePosition, self.m_driveVelocity
        )
        angleRotations = BaseStatusSignal.get_latency_compensated_value(
            self.m_steerPosition, self.m_steerVelocity
        )

        distance = Conversions.falconRotationsToMechanismMeters(
            driveRotations,
            SwerveConstants.MODULE_TYPE.wheelCircumference,
            1,
        )
        angle = Rotation2d.fromRotations(angleRotations)

        self.m_internalState.distance = distance
        self.m_internalState.angle = angle

        return self.m_internalState

    def getSignals(self) -> list[BaseStatusSignal]:
        """
        Returns the list of signals associated with the module.

        Returns:
        - The list of signals.
        """
        return self.m_signals
