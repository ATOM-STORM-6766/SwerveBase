import math

from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Twist2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.trajectory import TrapezoidProfile


class Util:
    EPSILON = 1e-12

    @staticmethod
    def boundRotation(rotation: Rotation2d) -> Rotation2d:
        return Rotation2d(rotation.radians())

    @staticmethod
    def boundRotation0_360(rotation: Rotation2d) -> Rotation2d:
        rotation = Util.boundRotation(rotation)
        if rotation < 0:
            return rotation + Rotation2d(math.tau)
        return rotation

    @staticmethod
    def get_tangent(start, end):
        dist = end - start
        return math.atan2(dist[1], dist[0])

    @staticmethod
    def epsilon_equals(a, b, epsilon=EPSILON):
        return (a - epsilon <= b) and (a + epsilon >= b)
