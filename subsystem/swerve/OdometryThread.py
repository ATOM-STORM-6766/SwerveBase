from threading import Thread

from phoenix6 import BaseStatusSignal, utils
from phoenix6.hardware import Pigeon2

from wpimath.filter import LinearFilter
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveDrive4Odometry, SwerveModulePosition

from subsystem.swerve.swerveModule import SwerveModule


class OdometryThread(Thread):
    m_allSignals: list[BaseStatusSignal]
    SuccessfulDaqs = 0
    FailedDaqs = 0
    ModuleCount: int

    lowpass = LinearFilter.movingAverage(50)
    lastTime = 0.0
    currentTime = 0.0
    averageLoopTime = 0.0

    m_modules: list[SwerveModule]
    m_modulePositions = [SwerveModulePosition() for _ in range(4)]
    m_odometry: SwerveDrive4Odometry
    m_pigeon: Pigeon2

    def __init__(
        self,
        odometry: SwerveDrive4Odometry,
        modules: list[SwerveModule],
        pigeon2: Pigeon2,
        modCount: int,
    ):
        super().__init__(daemon=True)
        # 4 signals for each module + 2 for Pigeon2
        self.ModuleCount = modCount
        self.m_modules = modules
        self.m_odometry = odometry
        self.m_pigeon2 = pigeon2
        self.m_allSignals = [BaseStatusSignal] * ((self.ModuleCount * 4) + 2)

        for i in range(self.ModuleCount):
            signals = self.m_modules[i].getSignals()
            self.m_allSignals[(i * 4) + 0] = signals[0]
            self.m_allSignals[(i * 4) + 1] = signals[1]
            self.m_allSignals[(i * 4) + 2] = signals[2]
            self.m_allSignals[(i * 4) + 3] = signals[3]

        self.m_allSignals[-2] = self.m_pigeon2.get_yaw()
        self.m_allSignals[-1] = self.m_pigeon2.get_angular_velocity_z_world()

    def run(self):
        # Make sure all signals update at around 250hz
        for sig in self.m_allSignals:
            sig.set_update_frequency(250)
        # Run as fast as possible, our signals will control the timing
        while True:
            # Synchronously wait for all signals in drivetrain
            status = BaseStatusSignal.wait_for_all(0.1, self.m_allSignals)
            self.lastTime = self.currentTime
            self.lastTime = self.currentTime
            self.currentTime = utils.get_current_time_seconds()
            self.averageLoopTime = self.lowpass.calculate(
                self.currentTime - self.lastTime
            )

            # Get status of the waitForAll
            if status.is_ok():
                self.SuccessfulDaqs += 1
            else:
                self.FailedDaqs += 1

            # Now update odometry
            for i in range(self.ModuleCount):
                # No need to refresh since it's automatically refreshed from the waitForAll()
                self.m_modulePositions[i] = self.m_modules[i].getPosition(False)

            yawDegrees = Rotation2d.fromDegrees(
                BaseStatusSignal.get_latency_compensated_value(
                    self.m_pigeon2.get_yaw(),
                    self.m_pigeon2.get_angular_velocity_z_world(),
                )
            )

            self.m_odometry.update(yawDegrees, self.m_modulePositions)

    def get_time(self):
        return self.averageLoopTime

    def get_successful_daqs(self):
        return self.SuccessfulDaqs

    def get_failed_daqs(self):
        return self.FailedDaqs
