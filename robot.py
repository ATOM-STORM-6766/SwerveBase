from phoenix6.signals import NeutralModeValue

from wpilib import TimedRobot, Timer
from commands2 import Command, CommandScheduler

from robotContainer import RobotContainer


class Robot(TimedRobot):
    autonomousCommand: Command
    robotContainer: RobotContainer

    coastTime = Timer()

    def robotInit(self) -> None:
        # Instantiate our RobotContainer. This will perform all our button bindings,
        # and put our autonomous chooser on the dashboard.
        self.robotContainer = RobotContainer()
        self.autonomousCommand = None

    def robotPeriodic(self) -> None:
        CommandScheduler.getInstance().run()

    def teleopInit(self) -> None:
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()

        self.robotContainer.getSwerve().setNeutralModes(
            NeutralModeValue.BRAKE, NeutralModeValue.BRAKE
        )

    def teleopPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        self.coastTime.reset()
        self.coastTime.start()

    def disabledPeriodic(self) -> None:
        if self.coastTime.get() == 5:
            self.robotContainer.getSwerve().setNeutralModes(
                NeutralModeValue.COAST, NeutralModeValue.COAST
            )
