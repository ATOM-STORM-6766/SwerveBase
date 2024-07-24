from wpilib.shuffleboard import ComplexWidget, Shuffleboard
from wpilib import SendableChooser

from commands2 import Command

from shuffleboard.ShuffleboardTabBase import ShuffleboardTabBase, ShuffleboardTab


class MatchTab(ShuffleboardTabBase):
    m_autoChooser = SendableChooser()
    m_autoChooserEntry: ComplexWidget

    def createEntries(self):
        self.m_tab = Shuffleboard.getTab("Match")
        self.m_autoChooserEntry = self.createSendableEntry(
            self.m_tab.add(
                "Auto Chooser",
                self.m_autoChooser,
            ),
            self.EntryProperties(0, 0, 2, 1),
        )

    @staticmethod
    def getAutoChooser() -> SendableChooser:
        return MatchTab.m_autoChooser
