from wpilib import reportError, DriverStation

from commands2 import Command, InstantCommand, PrintCommand

from auto.Autonomous import Autonomous
from auto.Routines import Routines

from commands.swerve import TeleopSwerver
from controlboard import Controlboard
from shuffleboard.ShuffleboardTabManager import ShuffleboardTabManager
from subsystem.swerve.swerve import Swerve


class RobotContainer:

    # Subsystems
    m_swerve = Swerve()
    m_shuffleboardTabManager = ShuffleboardTabManager()
    m_alliance = DriverStation.getAlliance()

    def __init__(self) -> None:
        """Configures the basic robot systems, such as Shuffleboard, autonomous, default commands, and button bindings."""
        self.m_shuffleboardTabManager.addTabs(True)
        self.configButtonBindings()
        self.configDefaultCommands()
        self.configAuto()

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

    def configAuto(self):
        """
        configures auto.

        - Configure default auto and named commands with `configure(Command defaultAuto, NamedCommand... namedCommands)`. This step must be done first.
        - Add auto routines with `addCommands(Command... commands)`.
        """
        Autonomous.configure(
            Command().withName("Do Nothing"),
            Autonomous.PPEvent(
                "ExampleEvent", PrintCommand("This is an example event :)")
            ),
        )

        Autonomous.addRoutines(Routines.exampleAuto().withName("Example Auto"))

    def getAutonomousCommand(self) -> Command:
        """
        retrieves the selected autonomous command from the autonomous chooser.

        Returns:
        - The selected autonomous command.
        """
        print("[Auto] Selected auto routine: " + Autonomous.getSelected().getName())
        return Autonomous.getSelected()

    @staticmethod
    def getSwerve() -> Swerve:
        """
        Retrieves the Swerve subsystem.

        return
        - The Swerve subsystem.
        """
        return RobotContainer.m_swerve

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
            reportError("[ERROR] Could not retrieve Alliance", True)
            return DriverStation.Alliance.kBlue
