# Copyright 2020 Hochschule Luzern - Informatik
# Author: Simon van Hemert <simon.vanhemert@hslu.ch>
# Author: Peter Sollberger <peter.sollberger@hslu.ch>


class PIDController:
    """
    Implements a PID controller.
    """

    def __init__(self):
        # Initialize variables
        self.reference_value = 415  # Reference (e.g. position in mm)
        self.error_linear = self.reference_value  # Initial error
        self.error_integral = 0
        self.anti_windup = 1023  # Anti-windup for Integrator, 1023 equals 5V = max speed

        # PID constants:
        self.kp = 180 / 1023.0 * 36
        self.Tn = 20
        self.Tv = 0

    def reset(self):
        """
        Restore controller with initial values.
        """
        self.error_linear = 0
        self.error_integral = 0

    def calculate_controller_output(self, actual_value):
        """
        Calculate next target values with the help of a PID controller.
        """
        sampling_time = 0.01
        error_linear_old = self.error_linear
        self.error_linear = self.reference_value - actual_value

        self.error_integral += self.error_linear * sampling_time
        integral_limit = self.anti_windup / (self.kp / self.Tn)
        self.error_integral = max(min(self.error_integral, integral_limit), -integral_limit)

        error_derivative = (self.error_linear - error_linear_old) / sampling_time

        p_part = self.kp * self.error_linear
        i_part = self.kp / self.Tn * self.error_integral
        d_part = self.kp * self.Tv * error_derivative

        # Save the three parts of the controller in a vector
        pid_actions = [p_part, i_part, d_part]
        # The output speed is the sum of the parts, 1023 equals 5V = max output
        controller_output = max(min(sum(pid_actions), self.anti_windup), -self.anti_windup)

        return int(controller_output), pid_actions
