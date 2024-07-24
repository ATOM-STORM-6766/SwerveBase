from pathplannerlib.auto import PathPlannerAuto
from commands2 import command


class Routines:
    @staticmethod
    def exampleAuto():
        """
        returns a full autonomous command created in the PathPlanner application.
        
        It uses Named Commands to trigger commands along the path.

        Returns:
        - A full autonomous command.
        """
        return PathPlannerAuto("ExampleAuto")
