from phoenix6 import BaseStatusSignal, StatusSignal
from phoenix6.controls import (
    PositionVoltage,
    VelocityTorqueCurrentFOC,
    VoltageOut,
    NeutralOut,
)
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.signals import NeutralModeValue

from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from lib.config.DeviceConfig import DeviceConfig
from lib.math.Conversions import Conversions
from lib.pid.PIDConstants import PIDConstants

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
        self.m_angleEncoder = CANcoder(constants.encoderID(), Ports.CAN_BUS_NAME)
        self.configAngleEncoder()

        self.m_steerMotor = TalonFX(constants.steerMotorID(), Ports.CAN_BUS_NAME)
        self.configSteerMotor()

        self.m_driveMotor = TalonFX(constants.driveMotorID(), Ports.CAN_BUS_NAME)
        self.configDriveMotor()

        self.m_drivePosition = self.m_driveMotor.getPosition()
        self.m_driveVelocity = self.m_driveMotor.getVelocity()
        self.m_steerPosition = self.m_steerMotor.getPosition()
        self.m_steerVelocity = self.m_steerMotor.getVelocity()

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

    def getModuleNumber(self):
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
        control=NeutralOut()
        self.m_steerMotor.get_control_mode().value

    def setDriveNeutralMode(self, mode):
        self.m_driveMotor.setNeutralMode(mode)
