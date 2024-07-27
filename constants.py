from dataclasses import dataclass
from enum import Enum

from phoenix6.signals import InvertedValue, NeutralModeValue
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig

from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath import units

from lib.pid import ScreamPIDConstants
from lib.util import COTSFalconSwerveConstants

LOOP_TIME_SEC = 0.02
""" Robot loop time """
LOOP_TIME_HZ = 1 / LOOP_TIME_SEC


class MotionMagicConstants:
    pass


class Ports:
    """
    Possible CAN bus strings are:
    "rio" for the native roboRIO CAN bus
    CANivore name or serial number
    "*" for any CANivore seen by the program
    """

    CAN_BUS_NAME = "CANivore"
    PIGEON_ID = 10


class ShuffleboardConstants:
    # For updating values like PID from Shuffleboard
    UPDATE_SWERVE = True


class SwerveConstants:

    # Drivebase Constants
    TRACK_WIDTH = 0.51685
    """Distance from left wheels to right wheels"""
    WHEEL_BASE = 0.51685
    """Distance from front wheels to back wheels"""

    # Gyro Constants
    GYRO_INVERT = False
    """Always ensure gyro reads CCW+ CW-"""

    # Swerve Kinematics
    MAX_SPEED = 5.7349  # m/s
    MAX_ANGULAR_VELOCITY = 8.0  # rad/s

    # Swerve Kinematics
    # No need to ever change this unless there are more than four modules.
    KINEMATICS = SwerveDrive4Kinematics(
        Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
    )

    # Selected Module Constants
    MODULE_TYPE: COTSFalconSwerveConstants = COTSFalconSwerveConstants.SDSMK4i(
        COTSFalconSwerveConstants.driveGearRatios.L3
    )

    # Swerve Heading Correction
    HEADING_CONSTANTS = ScreamPIDConstants(0.1, 0.0, 0.001)
    CORRECTION_TIME_THRESHOLD = 0.2

    # PathPlanner Constants
    PATH_TRANSLATION_CONSTANTS = ScreamPIDConstants(25, 0.0, 0.0)  # TODO ROBOT SPECIFIC
    PATH_ROTATION_CONSTANTS = ScreamPIDConstants(45, 0.0, 0.0)

    PATH_FOLLOWER_CONFIG = HolonomicPathFollowerConfig(
        PATH_TRANSLATION_CONSTANTS.toPathPlannerPIDConstants(),
        PATH_ROTATION_CONSTANTS.toPathPlannerPIDConstants(),
        MAX_SPEED,
        Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2).norm(),
        ReplanningConfig(),
        LOOP_TIME_SEC,
    )


class DriveConstants:
    # Gear Ratio
    GEAR_RATIO = SwerveConstants.MODULE_TYPE.driveGearRatio

    # Neutral Mode
    NEUTRAL_MODE = NeutralModeValue.BRAKE

    # Motor Invert
    MOTOR_INVERT = SwerveConstants.MODULE_TYPE.driveMotorInvert

    # Current Limit Constants
    SUPPLY_CURRENT_LIMIT = 35
    SUPPLY_CURRENT_THRESHOLD = 60
    SUPPLY_TIME_THRESHOLD = 0.1
    CURRENT_LIMIT_ENABLE = True
    SLIP_CURRENT = 400

    # Ramps
    OPEN_LOOP_RAMP = 0.25
    CLOSED_LOOP_RAMP = 0.0

    # PID Constants
    KP = 0.12  # TODO ROBOT SPECIFIC
    KI = 0.0
    KD = 0.0
    KF = 0.0
    PID_CONSTANTS = ScreamPIDConstants(KP, KI, KD, KF)

    # Feedforward Constants
    KS = 0.32  # TODO ROBOT SPECIFIC
    KV = 1.51
    KA = 0.27


class SteerConstants:
    # Gear Ratio
    GEAR_RATIO = SwerveConstants.MODULE_TYPE.steerGearRatio

    # Motor Invert
    MOTOR_INVERT = SwerveConstants.MODULE_TYPE.steerMotorInvert

    # Neutral Modes
    NEUTRAL_MODE = (
        NeutralModeValue.BRAKE
    )  # TODO CHANGE TO BRAKE AFTER MEASURING OFFSETS

    # Current Limits
    SUPPLY_CURRENT_LIMIT = 25
    SUPPLY_CURRENT_THRESHOLD = 40
    SUPPLY_TIME_THRESHOLD = 0.1
    CURRENT_LIMIT_ENABLE = True

    # PID Constants
    KP = SwerveConstants.MODULE_TYPE.steerKP
    KI = SwerveConstants.MODULE_TYPE.steerKI
    KD = SwerveConstants.MODULE_TYPE.steerKD
    KF = SwerveConstants.MODULE_TYPE.steerKF
    PID_CONSTANTS = ScreamPIDConstants(KP, KI, KD, KF)


class ModuleConstants:
    @dataclass
    class SwerveModuleConstants:
        driveMotorID: int
        steerMotorID: int
        encoderID: int
        angleOffset: Rotation2d

    class ModuleLocation(Enum):
        FRONT_LEFT = 0
        FRONT_RIGHT = 1
        BACK_LEFT = 2
        BACK_RIGHT = 3

        def getNumber(self):
            return self.value

        def toString(self) -> str:
            return self.name

    MODULE_0 = SwerveModuleConstants(2, 3, 11, Rotation2d.fromRotations(-0.359))  # ROBOT SPECIFIC

    MODULE_1 = SwerveModuleConstants(8, 9, 14, Rotation2d.fromRotations(-0.735))  # ROBOT SPECIFIC

    MODULE_2 = SwerveModuleConstants(4, 5, 12, Rotation2d.fromRotations(-0.187))  # ROBOT SPECIFIC

    MODULE_3 = SwerveModuleConstants(6, 7, 13, Rotation2d.fromRotations(-0.662))  # ROBOT SPECIFIC
