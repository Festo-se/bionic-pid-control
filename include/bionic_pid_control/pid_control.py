#!/usr/bin/env python3

import time

class PID:
    
    def __init__(self, p=0.2, i=0.0, d=0.0):

        self.reset()

        self.kp = p
        self.ki = i
        self.kd = d

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = time.time()

    def reset(self):
        """
        Reset the PID parameters
        """

        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 0.5

        self.output = 0.0

    def update(self, actual, desired):
        """
        Updates the PID value for the given reference feedback        
        """
        
        error = desired - actual

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        
        if (delta_time >= self.sample_time):
            self.p_term = self.kp * error
            self.i_term += error * delta_time
            
            if (self.i_term < -self.windup_guard):
                self.i_term = -self.windup_guard
            elif (self.i_term > self.windup_guard):
                self.i_term = self.windup_guard

            self.d_term = 0.0
            if delta_time > 0:
                self.d_term = delta_error / delta_time
            
            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            return self.p_term + (self.ki * self.i_term) + (self.kd * self.d_term)

    def set_kp(self, proportional_gain):
        """
        Determines how aggressively the PID reacts to the current error with setting Proportional Gain
        """

        self.kp = proportional_gain

    def set_ki(self, integral_gain):
        """
        Determines how aggressively the PID reacts to the current error with setting Integral Gain
        """

        self.ki = integral_gain

    def setKd(self, derivative_gain):
        """
        Determines how aggressively the PID reacts to the current error with setting Derivative Gain
        """

        self.kd = derivative_gain

    def set_windup(self, windup):
        """
        Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """

        self.windup_guard = windup

    def set_sample_time(self, sample_time):
        """
        PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """

        self.sample_time = sample_time
