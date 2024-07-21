from typing import overload

from phoenix6.hardware import Pigeon2
from phoenix6.signals import NeutralModeValue

from pathplannerlib.auto import AutoBuilder

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)

from wpilib import DriverStation

from commands2 import Subsystem

from lib.config import DeviceConfig
from lib.pid import ScreamPIDConstants

import constants
from constants import Ports
from constants import SwerveConstants
from constants import ModuleConstants

from robotContainer import RobotContainer

from subsystem.swerve import SwerveModule
from subsystem.swerve import OdometryThread


class Swerve(Subsystem):
    """
    A swerve drive subsystem.
    This class provides methods for high-level control of the swerve drivetrain.
    """

    m_pigeon2: Pigeon2
    m_swerveModules: list[SwerveModule]
    m_odometry: SwerveDrive4Odometry
    m_odometryThread: OdometryThread
    m_currentSpeeds = ChassisSpeeds()

    def __init__(self):
        self.m_pigeon2 = Pigeon2(Ports.PIGEON_ID, Ports.CAN_BUS_NAME)
        self.configGyro()

        """
        Initializes an array of SwerveModule objects with their respective names,IDs, and constants.
        This array represents the robot's four swerve modules.
        If there are multiple sets of modules, swap out the constants for the module in use.
        """
        self.m_swerveModules = [
            SwerveModule(
                ModuleConstants.ModuleLocation.FRONT_LEFT, ModuleConstants.MODULE_0
            ),
            SwerveModule(
                ModuleConstants.ModuleLocation.FRONT_RIGHT, ModuleConstants.MODULE_1
            ),
            SwerveModule(
                ModuleConstants.ModuleLocation.BACK_LEFT, ModuleConstants.MODULE_2
            ),
            SwerveModule(
                ModuleConstants.ModuleLocation.BACK_RIGHT, ModuleConstants.MODULE_3
            ),
        ]

        """
        Configures the odometry, which requires the kinematics, gyro reading, and module positions.
        It uses these values to estimate the robot's position on the field.
        """
        self.m_odometry = SwerveDrive4Odometry(
            SwerveConstants.KINEMATICS,
            self.getYaw(),
            self.getModulePositions(),
            Pose2d(),
        )

        """
        Configures the odometry thread, which uses the odometry to update the robot's position.
        """
        self.m_odometryThread = OdometryThread(
            self.m_odometry,
            self.m_swerveModules,
            self.m_pigeon2,
            len(self.m_swerveModules),
        )
        self.m_odometryThread.start()

        """
        Configures the AutoBuilder for holonomic mode.
        The AutoBuilder uses methods from this class to follow paths.
        """
        AutoBuilder.configureHolonomic(
            self.getPose,
            self.resetPose,
            self.robotRelativeSpeeds,
            self.setChassisSpeeds,
            SwerveConstants.PATH_FOLLOWER_CONFIG,
            lambda: RobotContainer.getAlliance() != DriverStation.Alliance.kBlue,
            self,
        )

    def getPose(self) -> Pose2d:
        """
        retrieves the estimated pose of the odometry.

        Returns:
        - The current pose of the odometry.
        """
        return self.m_odometry.getPose()

    def getModules(self) -> list[SwerveModule]:
        """
        returns an array composed of the swerve modules in the system.

        Returns:
        - The array of `SwerveModule` objects.
        """
        return self.m_swerveModules

    def getModuleStates(self) -> list[SwerveModuleState]:
        """
        returns an array composed of the state of each module in the system.

        Returns:
        - The array of `SwerveModuleState` objects.
        """
        states = list(4)
        for module in self.m_swerveModules:
            states[module.getModuleNumber()] = module.getState(True)
        return states

    def getModulePositions(self) -> list[SwerveModulePosition]:
        """
        returns an array composed of the position of each module in the system.

        Returns:
        - The array of `SwerveModulePosition` objects.
        """
        positions = list(4)
        for module in self.m_swerveModules:
            positions[module.getModuleNumber()] = module.getPosition(True)
        return positions

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        """
        returns the current robot-centric ChassisSpeeds.
        - Used by AutoBuilder.

        Returns:
        - The current robot-centric ChassisSpeeds.
        """
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            self.m_currentSpeeds, self.getYaw()
        )

    def getYaw(self) -> Rotation2d:
        """
        returns the yaw rotation in degrees. If `invertGyro` is set to true, the yaw rotation is inverted.

        Returns:
        - The yaw rotation as a `Rotation2d`.
        """
        return (
            Rotation2d.fromDegrees(self.m_pigeon2.get_yaw())
            if not SwerveConstants.GYRO_INVERT
            else Rotation2d.fromDegrees(-self.m_pigeon2.get_yaw())
        )

    def getGyro(self) -> Pigeon2:
        """
        returns the gyro object.
        - Used by SwerveTab to display to Shuffleboard.

        Returns:
        - The gyro object.
        """
        return self.m_pigeon2

    def zeroGyro(self):
        """
        Zeros the gyro.
        """
        self.m_pigeon2.set_yaw(0)

    def robotRelativeSpeeds(self, translation: Translation2d, angularVel: float):
        """
        returns a new robot-relative ChassisSpeeds based on the given inputs.

        Parameters:
        - `translation`: A Translation2d representing the desired movement (m/s) in the x and y directions.
        - `angularVel`: The desired angular velocity (rad/s).

        Returns:
        - The calculated ChassisSpeeds.
        """
        return ChassisSpeeds.discretize(
            translation.X(), translation.Y(), angularVel, constants.LOOP_TIME_SEC
        )

    def fieldRelativeSpeeds(self, translation: Translation2d, angularVel: float):
        """
        returns a new field-relative ChassisSpeeds based on the given inputs.

        Parameters:
        - `translation`: A Translation2d representing the desired movement (m/s) in the x and y directions.
        - `angularVel`: The desired angular velocity (rad/s).

        Returns:
        - The calculated ChassisSpeeds.
        """
        speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.X(), translation.Y(), angularVel, self.getYaw()
        )
        return ChassisSpeeds.discretize(speeds, constants.LOOP_TIME_SEC)

    @overload
    def setChassisSpeeds(self, chassisSpeeds: ChassisSpeeds, isOpenloop: bool):
        """
        sets the ChassisSpeeds to drive the robot. You can use predefined methods such as `robotSpeeds` or create a new ChassisSpeeds object.

        Parameters:
        - `chassisSpeeds`: The ChassisSpeeds to generate states for.
        - `isOpenLoop`: Whether the ChassisSpeeds is open loop (Tele-Op driving), or closed loop (Autonomous driving).
        """
        swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(
            chassisSpeeds
        )
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, SwerveConstants.MAX_SPEED
        )
        self.m_currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(
            swerveModuleStates
        )

        for mod in self.m_swerveModules:
            mod.set(swerveModuleStates[mod.getModuleNumber()], isOpenloop)

    @overload
    def setChassisSpeeds(self, chassisSpeeds: ChassisSpeeds):
        """
        sets the ChassisSpeeds to drive the robot. It defaults to closed loop control.
        - Used by AutoBuilder.

        Parameters:
        - `chassisSpeeds`: The ChassisSpeeds to generate states for.
        """
        self.setChassisSpeeds(chassisSpeeds, False)

    def setNeutralModes(self, driveMode: NeutralModeValue, steerMode: NeutralModeValue):
        """sets the neutral mode of the motors.

        Parameters:
        - `driveMode`: The NeutralModeValue to set the drive motor to. Use `NeutralModeValue.Brake` or `NeutralModeValue.Coast`.
        - `steerMode`: The NeutralModeValue to set the steer motor to. Use `NeutralModeValue.Brake` or `NeutralModeValue.Coast`.
        """
        for mod in self.m_swerveModules:
            mod.setDriveNeutralMode(driveMode)
            mod.setSteerNeutralMode(steerMode)

    def calculateHeadingCorrection(
        self, currentAngle: float, lastAngle: float
    ) -> float:
        """calculates the hold value based on the provided current and last angles.

        Parameters:
        - `currentAngle`: The current angle measurement.
        - `lastAngle`: The desired angle to calculate towards.

        Returns:
        - The calculated value from the heading controller.
        """
        return SwerveConstants.HEADING_CONSTANTS.toPIDController().calculate(
            currentAngle, lastAngle
        )

    def resetPose(self, pose: Pose2d):
        """
        Resets the robot's pose.
        """
        self.m_odometry.resetPosition(self.getYaw(), self.getModulePositions(), pose)

    def configGyro(self):
        """
        Configures the gyro.
        See DeviceConfig for more information.
        """
        DeviceConfig.configurePigeon2(
            "Swerve Pigeon",
            self.m_pigeon2,
            DeviceConfig.swervePigeonConfig(),
            constants.LOOP_TIME_HZ,
        )

    def configDrivePID(self, constants: ScreamPIDConstants):
        """
        configures all module drive motors with the given constants.

        Parameters:
        - `constants`: The `ScreamPIDConstants` to be applied.
        """
        for module in self.m_swerveModules:
            module.configDriveMotorPID(constants)
