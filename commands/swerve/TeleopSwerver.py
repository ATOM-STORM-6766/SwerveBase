from wpimath.geometry import Rotation2d, Translation2d
from wpilib import Timer

from commands2 import Command

from constants import SwerveConstants
from controlboard import Controlboard
from subsystem.swerve import Swerve


class TeleopSwerver(Command):
    swerve: Swerve
    translationSup: list[callable[[], float]]
    rotationSup: callable[[], float]
    fieldRelativeSup: callable[[], bool]
    lastAngle: Rotation2d
    correctionTimer = Timer()

    def __init__(
        self,
        swerve:Swerve,
        translationSup: list[callable[[], float]],
        rotationSup: callable[[], float],
        fieldRelativeSup: callable[[], bool],
    ):
        """
        Constructs a TeleopSwerve command with the given parameters.

        Parameters:
        - `swerve`: The Swerve subsystem to control.
        - `translationSup`: A supplier array for the translation value.
        - `strafeSup`: A supplier for the strafe value.
        - `rotationSup`: A supplier for the rotation value.
        - `fieldRelativeSup`: A supplier for the drive mode. Robot relative = false; Field relative = true
        """
        self.swerve = swerve
        self.addRequirements(swerve)

        self.translationSup = translationSup
        self.rotationSup = rotationSup
        self.fieldRelativeSup = fieldRelativeSup

    def initialize(self):
        self.correctionTimer.stop()
        self.correctionTimer.reset()
        self.lastAngle = self.swerve.getYaw()

    def execute(self):
        """
        Executes the swerve drive command.
        Passes the translation, strafe, and rotation values to the swerve subsystem to drive the robot.
        """
        translationVal = (
            Translation2d(self.translationSup[0](), self.translationSup[1]())
            * SwerveConstants.MAX_SPEED
        )
        rotationVal = self.getRotation(self.rotationSup())
        fieldRelativeVal = self.fieldRelativeSup()

        if Controlboard.getZeroGyro():
            self.lastAngle = Rotation2d.fromDegrees(0.0)

        self.swerve.setChassisSpeeds(
            (
                self.swerve.fieldRelativeSpeeds(translationVal, rotationVal)
                if fieldRelativeVal
                else self.swerve.robotRelativeSpeeds(translationVal, rotationVal)
            ),
            True,
        )

    def getRotation(self, current):
        """Checks if the swerve drive should start heading correction.

        If no manual input is given for a time, this method will return the rotation value required to maintain the current heading.

        Parameters:
        - `current`: The current rotation value.

        Returns:
        - The determined rotation value.
        """
        rotating = abs(current) > 0

        if rotating:
            self.correctionTimer.reset()
            return current * SwerveConstants.MAX_ANGULAR_VELOCITY

        self.correctionTimer.start()

        if self.correctionTimer.get() <= SwerveConstants.CORRECTION_TIME_THRESHOLD:
            self.last_angle = self.swerve.getYaw()

        if self.correctionTimer.hasElapsed(SwerveConstants.CORRECTION_TIME_THRESHOLD):
            return self.swerve.calculateHeadingCorrection(
                self.swerve.getYaw().degrees(), self.last_angle.degrees()
            )

        return current * SwerveConstants.MAX_ANGULAR_VELOCITY
