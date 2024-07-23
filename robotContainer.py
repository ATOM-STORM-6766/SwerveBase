import wpilib
from wpilib import DriverStation

from commands2 import Command, InstantCommand, PrintCommand


from commands.swerve import TeleopSwerver
from controlboard import Controlboard
from subsystem.swerve.swerve import Swerve


class RobotContainer:

    # Subsystems
    m_swerve = Swerve()
    # m_shuffleboardTabManager = ShuffleboardTabManager()

    m_alliance = DriverStation.getAlliance()

    def __init__(self) -> None:
        """Configures the basic robot systems, such as Shuffleboard, autonomous, default commands, and button bindings."""
        self.configButtonBindings()
        self.configDefaultCommands()

    def configButtonBindings(self) -> None:
        """Configures button bindings from Controlboard."""
        Controlboard.getZeroGyro().onTrue(
            InstantCommand(lambda: self.m_swerve.zeroGyro())
        )

    def configDefaultCommands(self) -> None:
        """Sets the default command for the swerve subsystem."""
        self.m_swerve.setDefaultCommand(
            TeleopSwerver(
                self.m_swerve,
                Controlboard.getTranslation(),
                Controlboard.getRotation(),
                Controlboard.getFieldCentric(),
            )
        )

    def getSwerve(self) -> Swerve:
        """
        Retrieves the Swerve subsystem.

        return
        - The Swerve subsystem.
        """
        return self.m_swerve

    @staticmethod
    def getAlliance():
        """
        Retrieves the current Alliance as detected by the DriverStation.
        Use this opposed to `DriverStation.getAlliance()`.

        return
        - The current Alliance.
        """
        if RobotContainer.m_alliance is not None:
            return RobotContainer.m_alliance
        else:
            wpilib.reportError("[ERROR] Could not retrieve Alliance", True)
            return DriverStation.Alliance.kBlue
