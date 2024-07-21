class Conversions:
    @staticmethod
    def falconRotationsToMechanismDegrees(
        falconRotations: float, gearRatio: float
    ) -> float:
        """
        Parameters:
        - `falconRotations`: The rotations of the Falcon.
        - `gearRatio`: The gear ratio between the Falcon and the mechanism.

        Returns:
        - The degrees of rotation of the mechanism.
        """
        return falconRotations * 360.0 / gearRatio

    @staticmethod
    def degreesToFalconRotations(degrees: float, gearRatio: float) -> float:
        """
        Parameters:
        - `degrees`: The degrees of rotation of the mechanism.
        - `gearRatio`: The gear ratio between the Falcon and the mechanism.

        Returns:
        - The rotations of the Falcon.
        """
        return degrees * gearRatio / 360.0

    @staticmethod
    def falconRPSToMechanismRPM(falconRPS: float, gearRatio: float) -> float:
        """
        Parameters:
        - `rps`: Falcon rotations per second
        - `gearRatio`: gear ratio between Falcon and mechanism (set to 1 for Falcon RPM)

        Returns:
        - The RPM of the mechanism.
        """
        return falconRPS * 60.0 / gearRatio

    @staticmethod
    def rpmToFalconRPS(rpm: float, gearRatio: float) -> float:
        """
        Parameters:
        - `rpm`: The RPM of the mechanism.
        - `gearRatio`: Gear ratio between Falcon and mechanism (set to 1 for Falcon RPS).

        Returns:
        - The rotations per second of the Falcon.
        """
        return rpm * gearRatio / 60.0

    @staticmethod
    def falconRotationsToMechanismMeters(
        falconRotations: float, circumference: float, gearRatio: float
    ) -> float:
        """
        Parameters:
        - `falconRotations`: The rotations of the Falcon.
        - `circumference`: The circumference of the wheel.
        - `gearRatio`: The gear ratio between the Falcon and the mechanism.

        Returns:
        - The linear distance traveled by the wheel in meters.
        """
        return falconRotations / gearRatio * circumference

    @staticmethod
    def falconRPSToMechanismMPS(
        falconRPS: float, circumference: float, gearRatio: float
    ) -> float:
        """
        Parameters:
        - `falconRPS`: Falcon rotations per second
        - `circumference`: circumference of the wheel
        - `gearRatio`: gear ratio between Falcon and mechanism

        Returns:
        - mechanism linear velocity in meters per second.
        """
        wheelRPM = Conversions.falconRPSToMechanismRPM(falconRPS, gearRatio)
        return (wheelRPM * circumference) / 60

    @staticmethod
    def mpsToFalconRPS(
        velocity: float, circumference: float, gearRatio: float
    ) -> float:
        """
        Parameters:
        - `velocity`: velocity in meters per second
        - `circumference`: circumference of the wheel
        - `gearRatio`: gear ratio between Falcon and mechanism

        Returns:
        - Falcon rotations per second
        """
        wheelRPM = (velocity * 60) / circumference
        return Conversions.rpmToFalconRPS(wheelRPM, gearRatio)
