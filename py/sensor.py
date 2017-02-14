import numpy as np
import abc

class Sensor(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def get_value(self):
        pass

class Encoder(Sensor):

    def __init__(self, cpr):
        self.cpr = int(cpr)

    def get_value(self, var):
        pulses = int(round(self.cpr * var / (2 * np.pi)))
        return 2 * np.pi * pulses / self.cpr
