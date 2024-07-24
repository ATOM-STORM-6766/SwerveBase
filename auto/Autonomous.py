from dataclasses import dataclass

from pathplannerlib.auto import NamedCommands

from wpilib import reportWarning, SmartDashboard
from commands2 import Command
from shuffleboard.tabs.MatchTab import MatchTab


class Autonomous:
    """
    A utility class that contains predefined auto routines for use during the autonomous period.
    """

    m_autoChooser = MatchTab.getAutoChooser()
    configured = False

    @dataclass
    class PPEvent:
        name: str
        command: Command

    @staticmethod
    def configure(defaultCommand: Command, *ppEvents: PPEvent) -> None:
        if Autonomous.configured:
            reportWarning("Autonomous already configured", False)
            return

        for ppEvent in ppEvents:
            NamedCommands.registerCommand(ppEvent.name, ppEvent.command)

        Autonomous.m_autoChooser.setDefaultOption(
            defaultCommand.getName(), defaultCommand
        )
        Autonomous.configured = True

    @staticmethod
    def getSelected():
        return Autonomous.m_autoChooser.getSelected()

    @staticmethod
    def addRoutines(*routines: Command):
        for routine in routines:
            Autonomous.m_autoChooser.addOption(routine.getName(), routine)
