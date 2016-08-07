
import numpy as np
import copy
import logging
import math
import matplotlib.pyplot as plt
import motor

logging.basicConfig(level = logging.DEBUG)

def create_axes(title = None, xlabel = None, ylabel = None, legend = None):
    fig = plt.figure()
    ax = plt.gca()
    if title:
        ax.set_title(title)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    if legend:
        ax.legend(legend, loc = 'best', frameon = False)
    ax.spines['bottom'].set_linewidth(2)
    ax.spines['left'].set_linewidth(2)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')
    ax.tick_params(labelsize = 14)
    ax.yaxis.grid(True, which = 'major')
    ax.xaxis.grid(True, which = 'major')
    return fig, ax
  

class Signal(object):

    def __init__(self, data = None, on_new_data = None):
        self.on_new_data = on_new_data
        self.buffer = np.array([])
        if data is not None:
            self.buffer = np.append(self.buffer, data)
        
    def _new_data_callback(self, data):
        if self.on_new_data:
            self.on_new_data(data)     
                        
    def add_data(self, data):
        if self.buffer.size == 0:
            logging.debug('Buffer empty. Appending: %s', data)
            # We append to the array if it's zero. vstack won't allow for stacking
            # if arrays don't have the same dimentions.
            self.buffer = np.append(self.buffer, data)
        else:
            logging.debug('Buffer not empty. Stacking: %s', data)
            # There's data. We can try to stack now.
            self.buffer = np.vstack((self.buffer, data))
            
        self._new_data_callback(data)

    def has_data(self):
        return self.buffer.size > 0

    def get_data(self, n = 0):
        if n >= 0:
            n -= 1
            data = self.buffer[:n]
            self.buffer = np.delete(self.buffer, slice(0, n))
            return data
        else:
            raise ValueError('Number of data to get must be positive.')
            
    def get_timeseries(self, n = 0):
        data = self.get_data(n)
        return data[: ,0], data[: ,1:]

if __name__ == '__main__':

    ti = 0.
    tf = 5.
    ts = 0.02

    params = {
        'kf': 0.1, # Friction Coefficient.
        'kb': 0.01, # Back-emf Coefficient.
        'L': 0.5, # Armature Inductance.
        'R': 1., # Resistance.
        'J': 0.01, # Rotor Inertia.
        'kt': 0.01, # Torque Constant.
        'ts': ts # Sampling time.
    }

    x0 = {
        'position': 0.,
        'velocity': 0.,
        'current': 0.
    }

    m = motor.DCMotor(params = params, initial_conditions = x0)
    s = Signal()
    t = ti
    while t < tf - ts:
        t += ts
        x = m.step(ts, u = 100.)
        x = np.insert(x, 0, t)
        s.add_data(x)
    time, data = s.get_timeseries()
    #logging.debug('Signal data: %s', signal_data)
    fig, ax = create_axes()
    ax.plot(time, data)
    plt.show()

