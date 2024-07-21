from phoenix6.configs import Slot0Configs, Slot1Configs, Slot2Configs
from pathplannerlib.config import PIDConstants

from wpimath.controller import PIDController
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile


class ScreamPIDConstants:
    def __init__(self, p=0.0, i=0.0, d=0.0, f=0.0):
        self.kP = p
        self.kI = i
        self.kD = d
        self.kF = f
        self.period = 0.02
        self.minOutput = -1.0
        self.maxOutput = 1.0
        self.integralZone = 1.0
        self.maxIntegralAccumulator = float("inf")
        self.minIntegralAccumulator = float("-inf")

    def setPID(self, p, i, d):
        self.kP = p
        self.kI = i
        self.kD = d

    def setPIDF(self, p, i, d, f):
        self.kP = p
        self.kI = i
        self.kD = d
        self.kF = f

    def setPeriod(self, period):
        self.period = period

    def setP(self, p):
        self.kP = p

    def setI(self, i):
        self.kI = i

    def setD(self, d):
        self.kD = d

    def setF(self, f):
        self.kF = f

    def setIntegralZone(self, Izone):
        self.integralZone = Izone

    def setIntegralAccumulatorBounds(self, max, min):
        self.maxIntegralAccumulator = max
        self.minIntegralAccumulator = min

    def setOutputBounds(self, max, min):
        self.maxOutput = max
        self.minOutput = min

    def withPID(self, p, i, d):
        self.kP = p
        self.kI = i
        self.kD = d
        return self

    def withPIDF(self, p, i, d, f):
        self.kP = p
        self.kI = i
        self.kD = d
        self.kF = f
        return self

    def withPeriod(self, period):
        self.period = period
        return self

    def withP(self, p):
        self.kP = p
        return self

    def withI(self, i):
        self.kI = i
        return self

    def withD(self, d):
        self.kD = d
        return self

    def withF(self, f):
        self.kF = f
        return self

    def withIntegralZone(self, Izone):
        self.integralZone = Izone
        return self

    def withIntegralAccumulatorBounds(self, max, min):
        self.maxIntegralAccumulator = max
        self.minIntegralAccumulator = min
        return self

    def withOutputBounds(self, max, min):
        self.maxOutput = max
        self.minOutput = min
        return self

    def toSlot0Configs(self):
        config = Slot0Configs()
        config.k_p = self.kP
        config.k_i = self.kI
        config.k_d = self.kD
        config.k_v = self.kF
        return config

    def toSlot1Configs(self):
        config = Slot1Configs()
        config.k_p = self.kP
        config.k_i = self.kI
        config.k_d = self.kD
        config.k_v = self.kF
        return config

    def toSlot2Configs(self):
        config = Slot2Configs()
        config.k_p = self.kP
        config.k_i = self.kI
        config.k_d = self.kD
        config.k_v = self.kF
        return config

    def toPathPlannerPIDConstants(self):
        return PIDConstants(self.kP, self.kI, self.kD, self.integralZone)

    def toPIDController(self):
        return PIDController(self.kP, self.kI, self.kD, self.period)

    def toProfiledPIDController(self, constraints: TrapezoidProfile.Constraints):
        return ProfiledPIDController(
            self.kP,
            self.kI,
            self.kD,
            constraints,
            self.period,
        )

    def __eq__(self, other):
        if isinstance(other, ScreamPIDConstants):
            return (
                self.period == other.period
                and self.kP == other.kP
                and self.kI == other.kI
                and self.kD == other.kD
                and self.kF == other.kF
                and self.minOutput == other.minOutput
                and self.maxOutput == other.maxOutput
                and self.integralZone == other.integralZone
                and self.maxIntegralAccumulator == other.maxIntegralAccumulator
                and self.minIntegralAccumulator == other.minIntegralAccumulator
            )
        return False

    def __copy__(self):
        copy = ScreamPIDConstants()
        copy.period = self.period
        copy.kP = self.kP
        copy.kI = self.kI
        copy.kD = self.kD
        copy.kF = self.kF
        copy.minOutput = self.minOutput
        copy.maxOutput = self.maxOutput
        copy.integralZone = self.integralZone
        copy.maxIntegralAccumulator = self.maxIntegralAccumulator
        copy.minIntegralAccumulator = self.minIntegralAccumulator
        return copy
