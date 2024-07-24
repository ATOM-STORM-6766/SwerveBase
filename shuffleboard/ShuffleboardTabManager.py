from commands2 import Subsystem

import robotContainer
from shuffleboard.tabs.MatchTab import MatchTab
from shuffleboard.tabs.SwerveTab import SwerveTab
from shuffleboard.ShuffleboardTabBase import ShuffleboardTabBase


class ShuffleboardTabManager(Subsystem):
    """Manages tabs for Shuffleboard."""

    m_tabs: list[ShuffleboardTabBase] = []

    def addTabs(self, includeDebug: bool):
        """adds predefined tabs to Shuffleboard.

        Parameters:
        - `includeDebug`: Whether to include additional debug tabs.
        """
        self.m_tabs.append(MatchTab())

        if includeDebug:
            self.m_tabs.append(SwerveTab(robotContainer.RobotContainer.getSwerve()))

        for tab in self.m_tabs:
            tab.createEntries()

    def periodic(self) -> None:
        """Calls the periodic method for each Shuffleboard tab in the list of tabs."""
        for tab in self.m_tabs:
            tab.periodic()
