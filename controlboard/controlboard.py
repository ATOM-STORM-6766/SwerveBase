from wpimath import applyDeadband
from commands2 import InstantCommand
from commands2.button import CommandXboxController


class Controlboard:
    """
    A utility class that contains button bindings.
    Controlboard allows easy reference of custom button associations.
    """

    STICK_DEADBAND = 0.05

    driverController = CommandXboxController(0)

    fieldCentric = True

    @staticmethod
    def getTranslation():
        """retrieves the swerve translation from the driver controller.

        Returns:
        - A DoubleSupplier array representing the x and y values from the controller.
        """
        return [
            lambda: -applyDeadband(
                Controlboard.driverController.getLeftY(), Controlboard.STICK_DEADBAND
            ),
            lambda: -applyDeadband(
                Controlboard.driverController.getLeftX(), Controlboard.STICK_DEADBAND
            ),
        ]

    @staticmethod
    def getRotation():
        """
        retrieves the rotation value from the driver controller.

        Returns:
        - A DoubleSupplier representing the rotation.
        """
        return lambda: -applyDeadband(
            Controlboard.driverController.getRightX(), Controlboard.STICK_DEADBAND
        )

    @staticmethod
    def getZeroGyro():
        """
        retrieves whether to zero the gyro from the driver controller.

        Returns:
        - A Trigger representing the state of the start button.
        """
        return Controlboard.driverController.back()

    @staticmethod
    def getFieldCentric():
        """
        retrieves the current field-centric mode.

        Returns:
        - True if field-centric; false if robot-centric.
        """

        def switch_field_centric():
            Controlboard.fieldCentric = not Controlboard.fieldCentric

        Controlboard.driverController.start().onTrue(
            InstantCommand(switch_field_centric)
        )
        return lambda: Controlboard.fieldCentric
