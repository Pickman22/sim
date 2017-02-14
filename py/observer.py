import numpy as np
from dynamics import DynamicSystem
import logging
import abc

class Observer(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def step(self):
        pass

    @abc.abstractmethod
    def get_estimate(self):
        pass

class Differentiator(object):

    def __init__(self, ts, x = 0.):
        if ts <= 0.:
            raise ValueError('Sampling time must be positive.')
        self.ts = float(ts)
        self.x = x

    def differentiate(self, x = 0.):
        ''' Backwards differentiation: y[k] = (x[k] - x[k-1]) / Ts,
            where x is the input signal, y is the output signal, k is the
            current instant and Ts is the sampling period. '''
        dx = (x - self.x) / self.ts
        self.x = x
        return dx

class MotorPositionDifferentiator(Observer):

    def __init__(self, ts, x = None):
        assert(x.dtype == np.float64 and type(x) is np.ndarray)
        if x is None:
            self.x = np.zeros(3, 1)
        else:
            self.x = x
        self.differentiator = Differentiator(ts, x = x[0])
        self.ts = ts
        self.dx = np.array([np.nan, 0., np.nan]).reshape(3, 1)

    def step(self, x):
        # x holds all states of the motor. Only pass the velocity to the
        # differentiator.
        assert(type(x) is np.ndarray and x.dtype == np.float64)
        self.dx[1] = self.differentiator.differentiate(x[0])
        return self.get_estimate()

    def get_estimate(self):
        return self.dx

class MotorSlidingModeObserver(Observer):

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
        return self.get_estimate()

    def get_estimate(self):
        return np.array([np.nan,
                         self.veq,
                         np.nan]).reshape(3, 1)
