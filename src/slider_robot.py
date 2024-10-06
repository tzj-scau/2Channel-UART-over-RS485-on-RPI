# slider_robot.py

import math
from motor import Motor

class SliderRobot:
    def __init__(self, motor: Motor, reduction_ratio=19, wheel_diameter=34.0, counts_per_revolution=3200):
        self.motor = motor
        self.reduction_ratio = reduction_ratio
        self.wheel_diameter = wheel_diameter
        self.counts_per_revolution = counts_per_revolution

    def enable_motor(self):
        self.motor.enable_motor()

    def disable_motor(self):
        self.motor.enable_motor(enable=False)

    def move_to_position(self, position_mm, velocity_mm_per_s=50, relative=True):
        counts = self.mm_to_counts(position_mm)
        velocity_rpm = self.velocity_mm_per_s_to_rpm(velocity_mm_per_s)
        direction = 0 if counts >= 0 else 1
        counts = abs(counts)
        self.motor.set_position(direction, counts, velocity_rpm, relative=relative)

    def mm_to_counts(self, position_mm):
        linear_movement_per_rev = (math.pi * self.wheel_diameter) / self.reduction_ratio  # mm per motor revolution
        counts_per_mm = self.counts_per_revolution / linear_movement_per_rev
        counts = int(position_mm * counts_per_mm)
        return counts

    def velocity_mm_per_s_to_rpm(self, velocity_mm_per_s):
        linear_movement_per_rev = (math.pi * self.wheel_diameter) / self.reduction_ratio
        revs_per_s = velocity_mm_per_s / linear_movement_per_rev
        rpm = revs_per_s * 60
        return int(rpm)

    def read_position(self):
        counts = self.motor.read_position()
        if counts is not None:
            position_mm = self.counts_to_mm(counts)
            return position_mm
        else:
            return None

    def counts_to_mm(self, counts):
        linear_movement_per_rev = (math.pi * self.wheel_diameter) / self.reduction_ratio
        mm_per_count = linear_movement_per_rev / self.counts_per_revolution
        position_mm = counts * mm_per_count
        return position_mm

    def read_velocity(self):
        velocity_rpm = self.motor.read_velocity()
        if velocity_rpm is not None:
            velocity_mm_per_s = self.velocity_rpm_to_mm_per_s(velocity_rpm)
            return velocity_mm_per_s
        else:
            return None

    def velocity_rpm_to_mm_per_s(self, rpm):
        linear_movement_per_rev = (math.pi * self.wheel_diameter) / self.reduction_ratio
        revs_per_s = rpm / 60
        velocity_mm_per_s = revs_per_s * linear_movement_per_rev
        return velocity_mm_per_s
