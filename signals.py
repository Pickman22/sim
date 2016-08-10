import numpy as np
import threading
import copy
import logging
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
import time
import serial
import json
import sys
from abc import ABCMeta, abstractmethod

matplotlib.use('TkAgg') # <-- THIS MAKES IT FAST!

logging.basicConfig(level = logging.DEBUG)
logging.getLogger().setLevel(logging.ERROR)

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

    __metaclass__ = ABCMeta

    def __init__(self, names, on_new_data = None, on_connect = None, on_disconnect = None, rate = 10.):
        self.on_new_data = on_new_data
        self.on_connect = on_connect
        self.on_disconnect = on_disconnect   
        self.buffer = {name: [] for name in names}
        self.rate = rate
        self.names = names
        self.is_connected = False

    def _new_data_callback(self, data):
        if self.on_new_data:
            self.on_new_data(data)
            
    def _stream(self):
        raw_data = self._readline()
        try:
            json_data = json.loads(raw_data)
            self.add_dict_data(json_data)
            logging.debug('json: %s', json_data)
        except ValueError:
            logging.error('Could not read data: %s', raw_data)
        threading.Timer(self.rate / 1000., self._stream).start()
            
    def start(self):
        if self.is_connected:
            self._stream()
            if self.on_connect:
                self.on_connect()
        else:
            raise ValueError('Conection must be established first!')
            
    def stop(self):
        self.disconnect()
        if not self.is_connected:
            if self.on_disconnect:
                self.on_disconnect()
        else:
            raise ValueError('Connection could not be terminated!')
    
    @abstractmethod
    def _readline(self, *params):
        pass
    
    @abstractmethod 
    def connect(self, *params):
        pass
        
    @abstractmethod
    def disconnect(self, *params):
        pass
    
    def has_data(self, name):
        if self.buffer.has_key(name):
            return len(self.buffer[name])
        else:
            return -1

         
    def add_dict_data(self, data_dict):
        for name, data in data_dict.iteritems():
            self.add_data(name, data)
        self._new_data_callback(data_dict)

    def add_data(self, name, data):
        self.buffer[name].append(data)

    def get_data(self, name, n = 0):
        '''Returns @n amount of data in signal @name. If n == 0, all data in 
        the buffer is returned. if n < 0, a ValueError exception is raised.'''
        if n == 0:
            # Get copy of list. If data = self.buffer[name] gets a reference.
            # When the values are cleared, the returned values get cleared
            # as well.
            data = self.buffer[name][:]
            del self.buffer[name][:]
        elif n > 0:
            data = self.buffer[name][:n]
            del self[name][:n]
        elif n < 0:
            data = None
            raise ValueError('Cannot get negative amounts of data')
        return data
        
    def get_data_dict(self):
        data = self.buffer
        for name in self.buffer.iterkeys():
            del data[name][:]


class SerialSignal(Signal):

    def __init__(self, names, port = '/dev/ttyACM0', baudrate = 9600,
    timeout = 1., rate = 10., on_new_data = None, on_connect = None,
    on_disconnect = None):
        Signal.__init__(self, names, on_new_data, on_connect, on_disconnect)
        self.connect(port, baudrate, timeout)

    def connect(self, *params):
        if self.is_connected:
            return
        self.port = serial.Serial(params[0], params[1], timeout = params[2])
        if not self.port.is_open:
            self.port.open()
        self.is_connected = self.port.is_open
        
    def _readline(self):
        return self.port.readline()
    
    def disconnect(self, *params):
        if not self.is_connected:
            return
        self.port.close()
        self.is_connected = self.port.is_open
        
        
class RealTimePlot(object):

    def __init__(self, signal, legend = None, title = None, xlabel = None, ylabel = None,
    interval = 60, blit = True, xlim = 300., ylim = [-2., 2.],
    autoscroll = True, autosize = True, keep_data = False):
        self.fig, self.ax = create_axes()
        if title is not None:
            self.ax.set_title(title)
        if xlabel is not None:
            self.ax.set_xlabel(xlabel)
        if ylabel is not None:
            self.ax.set_ylabel(ylabel)
        if legend == None:
            legend = signal.names
        self.lines = [plt.plot([0.], [0.], label = label)[0] for label in legend]
        self.xlim = xlim
        self.ax.set_ylim(ylim)
        self.ax.set_xlim([0, xlim])
        self.autoscroll = autoscroll
        self.autosize = autosize
        self.signal = signal
        self.keep_data = keep_data
        self.animation = animation.FuncAnimation(self.fig, self.update, interval = interval, blit = blit)
        plt.legend()
        plt.show()
        
    def _discard_data(self):
        pass
        
    def _init_plot(self):
        pass
            
    def _handle_autoscroll(self, t):
        '''If autoscroll is specified, this function handles it. This should
        not be used by application code.'''
        if self.autoscroll:
            # numpy empty arrays have dimension 0!!!!
            # so we have to use size to take care of this.
            # Empty arrays have size 0.
            if t.ndim == 0 and t.size != 0:
                logging.error('%s', t.ndim)
                if t > self.xlim:
                    self.ax.set_xlim([t - self.xlim, t])
            elif t.ndim > 0:
                if t[-1] > self.xlim:    
                    self.ax.set_xlim([t[-1] - self.xlim, t[-1]])
            else:
                logging.error('Time vector is unknown type: %s', type(t))
               
    def _handle_autosize(self, t):
        pass
     
    def update(self, i):
        '''Gets called to redraw the plot. Should not be used by application
        code.'''
        idx = 0
        for name, line in zip(self.signal.names, self.lines):
            if self.signal.has_data(name) > 0:
                _, ly = line.get_data()
                ly = np.append(ly, self.signal.get_data(name))
                lx = np.arange(ly.size)
                line.set_data(lx, ly)
                idx = max(idx, lx[-1])
            else:
                pass
        self._handle_autoscroll(idx)
        #self.fig.canvas.update()
        #self.ax.figure.canvas.draw()
        return self.lines
    
if __name__ == '__main__':
    stream = SerialSignal(['x', 'y', 'z'], baudrate = 115200)
    stream.start()
    r = RealTimePlot(stream)




