from ntcore import GenericEntry
from wpilib.shuffleboard import ComplexWidget, Shuffleboard
from wpiutil import Sendable, SendableBuilder

from constants import ShuffleboardConstants, DriveConstants
from shuffleboard.ShuffleboardTabBase import ShuffleboardTabBase
from subsystem.swerve.swerve import Swerve


class SwerveTab(ShuffleboardTabBase):
    m_swerve: Swerve

    def __init__(self, swerve: Swerve):
        """A Shuffleboard tab for displaying and updating Swerve drive subsystem data."""
        self.m_swerve = swerve

    m_FLEncoder: GenericEntry
    m_FLIntegrated: GenericEntry

    m_FREncoder: GenericEntry
    m_FRIntegrated: GenericEntry

    m_BLEncoder: GenericEntry
    m_BLIntegrated: GenericEntry

    m_BREncoder: GenericEntry
    m_BRIntegrated: GenericEntry

    m_odometryX: GenericEntry
    m_odometryY: GenericEntry
    m_odometryYaw: ComplexWidget

    m_driveP: GenericEntry

    def createEntries(self):
        """
        This method creates number entries for various sensors related to the Swerve subsystem.

        These entries are used to display and update values on the Shuffleboard.

        Set `ShuffleboardConstants.UPDATE_SWERVE` to true for entries that get values.
        """
        self.m_tab = Shuffleboard.getTab("Swerve")

        self.m_FLEncoder = self.createNumberEntry(
            "FL Encoder", 0.0, self.EntryProperties(0, 0)
        )
        self.m_FLIntegrated = self.createNumberEntry(
            "FL Integrated", 0.0, self.EntryProperties(1, 0)
        )

        self.m_FREncoder = self.createNumberEntry(
            "FR Encoder", 0.0, self.EntryProperties(0, 1)
        )
        self.m_FRIntegrated = self.createNumberEntry(
            "FR Integrated", 0.0, self.EntryProperties(1, 1)
        )

        self.m_BLEncoder = self.createNumberEntry(
            "BL Encoder", 0.0, self.EntryProperties(0, 2)
        )
        self.m_BLIntegrated = self.createNumberEntry(
            "BL Integrated", 0.0, self.EntryProperties(1, 2)
        )

        self.m_BREncoder = self.createNumberEntry(
            "BR Encoder", 0.0, self.EntryProperties(0, 3)
        )
        self.m_BRIntegrated = self.createNumberEntry(
            "BR Integrated", 0.0, self.EntryProperties(1, 3)
        )

        self.m_odometryX = self.createNumberEntry(
            "Odometry X", 0.0, self.EntryProperties(2, 0)
        )
        self.m_odometryY = self.createNumberEntry(
            "Odometry Y", 0.0, self.EntryProperties(2, 1)
        )

        class Pigeon2(Sendable):
            def __init__(self, Pigeon2):
                super().__init__()
                self.m_Pigeon2 = Pigeon2

            def initSendable(self, builder: SendableBuilder) -> None:
                builder.setSmartDashboardType("Gyro")
                builder.addDoubleProperty(
                    "Value",
                    lambda: self.m_Pigeon2.get_yaw().value,
                    self.m_Pigeon2.set_yaw,
                )

        self.m_odometryYaw = self.createSendableEntry(
            self.m_tab.add("Odometry Angle", Pigeon2(self.m_swerve.getGyro())),
            self.EntryProperties(2, 2),
        )

        if ShuffleboardConstants.UPDATE_SWERVE:
            self.m_driveP = self.createNumberEntry(
                "Drive P Gain",
                DriveConstants.PID_CONSTANTS.kP,
                self.EntryProperties(3, 0),
            )

    def periodic(self):
        self.m_FLEncoder.setDouble(
            self.filter(self.m_swerve.getModules()[0].getEncoderAngle().degrees())
        )
        self.m_FLIntegrated.setDouble(
            self.filter(self.m_swerve.getModules()[0].getPosition(True).angle.degrees())
        )

        self.m_FREncoder.setDouble(
            self.filter(self.m_swerve.getModules()[1].getEncoderAngle().degrees())
        )
        self.m_FRIntegrated.setDouble(
            self.filter(self.m_swerve.getModules()[1].getPosition(True).angle.degrees())
        )

        self.m_BLEncoder.setDouble(
            self.filter(self.m_swerve.getModules()[2].getEncoderAngle().degrees())
        )
        self.m_BLIntegrated.setDouble(
            self.filter(self.m_swerve.getModules()[2].getPosition(True).angle.degrees())
        )

        self.m_BREncoder.setDouble(
            self.filter(self.m_swerve.getModules()[3].getEncoderAngle().degrees())
        )
        self.m_BRIntegrated.setDouble(
            self.filter(self.m_swerve.getModules()[3].getPosition(True).angle.degrees())
        )

        self.m_odometryX.setDouble(self.m_swerve.getPose().X())
        self.m_odometryY.setDouble(self.m_swerve.getPose().Y())

        if ShuffleboardConstants.UPDATE_SWERVE:
            self.m_swerve.configDrivePID(
                DriveConstants.PID_CONSTANTS.withP(self.m_driveP.get().getDouble())
            )
