import numpy as np
import abc

class DynamicSystem(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def dynamics(self):
        pass

    def step(self, u = 0.):
        self.dx = self.dynamics(u)
        self.x = self.dx * self.ts + self.x
        return self.x
