from dataclasses import dataclass

from wpimath.filter import LinearFilter
from ntcore import GenericEntry, Value
from wpiutil import Sendable
from wpilib.shuffleboard import WidgetType, ComplexWidget, ShuffleboardTab, SimpleWidget


class ShuffleboardTabBase:
    """
    Base class for Shuffleboard tabs.
    """

    m_tab: ShuffleboardTab
    _filter = LinearFilter.movingAverage(50)

    @dataclass
    class EntryProperties:
        xPos: int = None
        yPos: int = None
        width: int = None
        height: int = None

    @dataclass
    class Widget:
        type: WidgetType = None
        propertyMap: dict[str, Value] = None

    def createEntries(self):
        """
        Creates the entries for the Shuffleboard tab.
        """
        pass

    def createEntry(
        self,
        entry: SimpleWidget,
        entryProps: EntryProperties,
        widget: Widget,
    ) -> GenericEntry:
        if entryProps.xPos is not None and entryProps.yPos is not None:
            entry = entry.withPosition(entryProps.xPos, entryProps.yPos)
        if entryProps.width is not None and entryProps.height is not None:
            entry = entry.withSize(entryProps.width, entryProps.height)
        if widget.type is not None:
            entry = entry.withWidget(widget.type)
        if widget.propertyMap is not None:
            entry = entry.withProperties(widget.propertyMap)
        return entry.getEntry()

    def createNumberEntry(
        self,
        name: str,
        defaultValue: float,
        entryProps: EntryProperties,
        widget: Widget = Widget(),
    ) -> GenericEntry:
        entry = self.m_tab.add(name, defaultValue)
        return self.createEntry(entry, entryProps, widget)

    def createStringEntry(
        self,
        name: str,
        defaultValue: str,
        entryProps: EntryProperties,
        widget: Widget = Widget(),
    ) -> GenericEntry:
        entry = self.m_tab.add(name, defaultValue)
        return self.createEntry(entry, entryProps, widget)

    def createBooleanEntry(
        self,
        name: str,
        defaultValue: bool,
        entryProps: EntryProperties,
        widget: Widget = Widget(),
    ) -> GenericEntry:
        entry = self.m_tab.add(name, defaultValue)
        return self.createEntry(entry, entryProps, widget)

    def createSendableEntry(
        self,
        entry: ComplexWidget,
        entryProps: EntryProperties,
        propertyMap: dict[str, Value] = None,
    ) -> GenericEntry:

        if entryProps.xPos is not None and entryProps.yPos is not None:
            entry = entry.withPosition(entryProps.xPos, entryProps.yPos)
        if entryProps.width is not None and entryProps.height is not None:
            entry = entry.withSize(entryProps.width, entryProps.height)
        if propertyMap is not None:
            entry = entry.withProperties(propertyMap)
        return entry

    def periodic(self):
        pass

    def filter(self, number: float) -> float:
        return self._filter.calculate(round(number * 1000) / 1000)

    def getTab(self) -> ShuffleboardTab:
        return self.m_tab
