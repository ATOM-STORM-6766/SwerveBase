import math

from phoenix6.signals import InvertedValue, SensorDirectionValue

from wpimath import units


class COTSFalconSwerveConstants:
    def __init__(
        self,
        wheelDiameter,
        angleGearRatio,
        driveGearRatio,
        angleKP,
        angleKI,
        angleKD,
        angleKF,
        driveMotorInvert,
        angleMotorInvert,
        CANcoderInvert,
    ):
        self.wheelDiameter = wheelDiameter
        self.wheelCircumference = wheelDiameter * math.pi
        self.steerGearRatio = angleGearRatio
        self.driveGearRatio = driveGearRatio
        self.steerKP = angleKP
        self.steerKI = angleKI
        self.steerKD = angleKD
        self.steerKF = angleKF
        self.driveMotorInvert = driveMotorInvert
        self.steerMotorInvert = angleMotorInvert
        self.CANcoderInvert = CANcoderInvert

    @staticmethod
    def SDSMK4i(driveGearRatio) -> "COTSFalconSwerveConstants":
        wheelDiameter = 4.0 * 0.0254  # inches to meters

        # 21.42 : 1
        angleGearRatio = (150.0 / 7.0) / 1.0

        angleKP = 100.0
        angleKI = 0.0
        angleKD = 0.0
        angleKF = 0.0

        # COUNTER_CLOCKWISE_POSITIVE = false; CLOCKWISE_POSITIVE = true
        driveMotorInvert = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        angleMotorInvert = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        CANcoderInvert = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE

        return COTSFalconSwerveConstants(
            wheelDiameter,
            angleGearRatio,
            driveGearRatio,
            angleKP,
            angleKI,
            angleKD,
            angleKF,
            driveMotorInvert,
            angleMotorInvert,
            CANcoderInvert,
        )

    class driveGearRatios:
        # SDS MK3
        MK3_Standard = 8.16 / 1.0  # SDS MK3 - 8.16 : 1
        MK3_Fast = 6.86 / 1.0  # SDS MK3 - 6.86 : 1

        # MK4 Modules
        L1 = 8.14 / 1.0  # L1 - 8.14 : 1
        L2 = 6.75 / 1.0  # L2 - 6.75 : 1
        L3 = 6.12 / 1.0  # L3 - 6.12 : 1
        L4 = 5.14 / 1.0  # L4 - 5.14 : 1
