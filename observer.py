import numpy as np
from dynamics import DynamicSystem
import logging

class Differentiator(DynamicSystem):

    def __init__(self, ts, x0 = 0.):
        self.ts = float(ts)
        self.x = x0
        self.dx = 0.

    def dynamics(self, x = 0.):
        ''' Backwards differentiation: y[k] = (x[k] - x[k-1]) / Ts,
            where x is the input signal, y is the output signal, k is the
            current instant and Ts is the sampling period. '''
        self.dx = (x - self.x) / self.ts
        self.x = x
        return self.dx

    def step(self, x):
        return self.dynamics(x)

    def get_value(self):
        return self.dx


class MotorVelocityObserver(Differentiator):

    def __init__(self, ts, x0 = np.zeros([3, 1]).reshape(3, 1)):
        Differentiator.__init__(self, ts, x0 = x0[1])

    def step(self, x):
        # x holds all states of the motor. Only pass the velocity to the
        # differentiator.
        Differentiator.step(self, x[0])
        return self.get_value()

    def get_value(self):
        return np.array([None,
                         Differentiator.get_value(self),
                         None]).reshape(3, 1)

class MotorSlidingModeObserver(DynamicSystem):

    def __init__(self, p, ts, tau = 1e-2, x = 0.):
        self.ts = ts
        self.x = x
        self.p = p
        self.dx = 0.
        self.veq = 0.
        self.tau = tau
        self.z = 0.

    def dynamics(self, x):
        self.z = float(self.x - x)
        return -self.p * np.sign(self.z)

    def step(self, x):
        self.dx = self.dynamics(x[0])
        self.x = self.dx * self.ts + self.x
        self.veq = (self.ts / self.tau) * (self.dx - self.veq) + self.veq

        return self.get_value()

    def get_value(self):
        return np.array([None,
                         self.veq,
                         None]).reshape(3, 1)
