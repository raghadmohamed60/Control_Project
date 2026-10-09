import math

import numpy as np


class LateralPIDController:
    def __init__(
        self,
        kp=0.8,
        ki=0.02,
        kd=0.15,
        k_yaw=0.5,
        dt=0.1,
        max_steer_rad=math.radians(35.0),
        integral_limit=1.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.k_yaw = k_yaw
        self.dt = dt
        self.max_steer_rad = max_steer_rad
        self.integral_limit = integral_limit
        self.integral_cte = 0.0
        self.prev_cte = 0.0

    def compute_steering(self, cte, heading_err):
        error = cte
        self.integral_cte += error * self.dt
        self.integral_cte = np.clip(
            self.integral_cte,
            -self.integral_limit,
            self.integral_limit,
        )
        derivative = (error - self.prev_cte) / self.dt

        steering = (
            0.0
            - self.kp * error
            - self.ki * self.integral_cte
            - self.kd * derivative
            - self.k_yaw * heading_err
        )

        self.prev_cte = error
        steering = np.clip(
            steering,
            -self.max_steer_rad,
            self.max_steer_rad,
        )
        return float(steering)

    def reset(self):
        self.integral_cte = 0.0
        self.prev_cte = 0.0
