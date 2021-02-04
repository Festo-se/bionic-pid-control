import rospy
import numpy as np

class MotionControl:
    ''' PID controller which requires vectors as input '''
    def __init__(self, Kp, Ki, Kd, desired=np.array([]), outputMaximum=200000, outputOffset=400000, guard=300000):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.I_term = 0

        self.desired = desired
        self.last_time = rospy.get_time()
        self.last_error = 0

        self.offset = outputOffset
        self.maximum = outputMaximum
        self.windup_guard = guard 


    def tick(self):
        ''' Reset recorded time of previous update. Useful for manipulating
            time after initialization but before the control loop has started. '''
        self.last_time = rospy.get_time()


    def bound(self, value, boundary):
        ''' Limit the value with a boundary in negative and positive direction'''
        maxBound = boundary
        minBound = -boundary

        for element in range(len(value)):
            if (value[element] < -boundary):
                value[element] = -boundary
            elif (value[element] > boundary):
                value[element] = boundary

        return value


    def update(self, actual, desired=np.array([])):
        ''' Update the control output '''
        error = desired - actual
        #print(f"POS DESIRED: {desired}")
        #print(f"POS ERROR: {error}")

        current_time = rospy.get_time()
        delta_time = current_time - self.last_time
        delta_error = error - self.last_error

        # P
        P_term = error

        # I
        self.I_term += error * delta_time
        self.bound(self.I_term, self.windup_guard)

        # D
        if delta_time > 0:
            D_term = delta_error / delta_time
        else:
            D_term = 0

        self.last_error = error
        self.last_time = current_time

        output = self.Kp * P_term + self.Ki * self.I_term + self.Kd * D_term
        output = -output
        # output 0 = 400 000
        # output negative = 100 000 .. 399 999
        # output positive = 400 001 .. 600 000

        #print(f"PRE BOUND: {output}")
        self.bound(output, self.maximum)
        #print(f"POST BOUND: {output}")
        output += self.offset
        #print(f"POST OUTPUT: {output}")

        return output