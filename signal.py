
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
  
  
class Buffer(object):

    def __init__(self, data = None, on_new_data = None, on_err = None):
        self.data = data
        self.on_new_data = on_new_data
        self.on_err = on_err
        
    def add_data(self, data):
        if self.data is not None:
            try:
                self.data = np.vstack((self.data, data))
            except ValueError:
                logging.error('Data is stacked vertically!')
                self.data = None
                if self.on_err:
                    self.on_err()
                return
        else:
            self.data = data
        if self.on_new_data:
            self.on_new_data(self.data)
            
    def clear(self):
        self.data = None
            
    def get_data(self):
        if self.data is not None:
            tmp = self.data
            self.data = None
            return tmp
        else:
            return None

class Signal(object):

    def __init__(self, xdata = None, ydata = None, on_new_data = None, on_err = None):
        self.x_buffer = Buffer()
        self.y_buffer = Buffer()
        self.on_new_data = on_new_data
        self.on_err = on_err
        if xdata is not None and ydata is not None:
            try:
                
                self.add_data(xdata, ydata)
            except ValueError:
                logging.error('Invalid data!')
                self.x_buffer.clear()
                self.y_buffer.clear()
                self._err()
                
    def has_data(self):
        return self.x_buffer.data is not None and self.y_buffer.data is not None
                
    def has_valid_data(self):
        if self.has_data():
            return len(self.x_buffer.data) == len(self.y_buffer.data)
        else:
            # No data is valid data!
            return True
                
    def get_data(self):
        if self.has_valid_data():
            return self.x_buffer.get_data(), self.y_buffer.get_data()
        
    
    def add_data(self, xdata, ydata):
        logging.debug('xdata: %s, ydata: %s', len(xdata), len(ydata))
        if len(xdata) != len(ydata):
            raise ValueError('x-Axis data and y-Axis data must be the samen length')
        else:
            self.x_buffer.add_data(xdata)
            self.y_buffer.add_data(ydata)
            if self.on_new_data:
                self.on_new_data(xdata, ydata)
                    
    def _err(self):
        if self.on_err:
            self.on_err()
                 
    def clear(self):
        self.x_buffer.clear()
        self.y_buffer.clear()

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
    s = Buffer()
    S = Signal()
    t = ti
    T = [ti]
    s.add_data(m.x.T)
    while t < tf - ts:
        t += ts
        x = m.step(ts, u = 100.)
        s.add_data(x.T)
        T.append(t)
    data = s.get_data()
    S.add_data(T, data)
    signal_t, signal_x = S.get_data()
    logging.debug('Data should be None: %s', s.data)
    #logging.debug('Signal data: %s', signal_data)
    fig, ax = create_axes()
    ax.plot(signal_t, signal_x)
    #ax.plot(signal_data[0], signal_data[1])
    plt.show()

