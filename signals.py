import numpy as np
import threading
import copy
import logging
import math
import matplotlib.pyplot as plt
import motor
import matplotlib.animation as animation
import time

logging.basicConfig(level = logging.DEBUG)
logging.getLogger().setLevel(logging.INFO)


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
  

def random_data(*args):
    logging.debug('Entering random_data callback')
    signal, t = args
    t += 0.02
    data = np.array([t, np.random.random()])
    logging.debug('%s, %s', data[0], data[1:])
    signal.add_data(data)
    logging.debug('signal buffer: %s', signal.buffer)
    threading.Timer(0.1, random_data, (signal, t)).start()
    logging.debug('Leaving random_data callback')

class Signal(object):

    def __init__(self, data = None, on_new_data = None):
        self.on_new_data = on_new_data
        self.buffer = np.array([])
        if data is not None:
            self.buffer = np.append(self.buffer, data)
        
    def _new_data_callback(self, data):
        '''Callback when new data is added to the buffer'''
        if self.on_new_data:
            self.on_new_data(data)     
                        
    def add_data(self, data):
        '''Add @data vector to the buffer. Data is stacked vertically, so data
        has to be in form of row vectors, where the first element is the time
        vector'''
        if self.buffer.size == 0:
            # We append to the array if it's zero. vstack won't allow for stacking
            # if arrays don't have the same dimentions.
            logging.debug('Buffer empty. Apending: %s', data)
            self.buffer = np.append(self.buffer, data)
        else:
            # There's data. We can try to stack now.
            logging.debug('Buffer not empty. Stacking: %s', data)
            self.buffer = np.vstack((self.buffer, data))
            
        self._new_data_callback(data)

    def has_data(self):
        return self.buffer.size > 0

    def get_data(self, n = 0):
        '''Returns @n amount of data. If n == 0, all data in the buffer is returned.
        if n < 0, a ValueError exception is raised.'''
        if n == 0:
            data = self.buffer
            self.buffer = np.array([])
        elif n > 0:
            data = self.buffer[:n]
            self.buffer = np.delete(self.buffer, slice(0, n))
        elif n < 0:
            data = None
            raise ValueError('Cannot get negative amounts of data')
        logging.debug('get_data: %s', data)
        return data

    def get_timeseries(self, n = 0):
        '''Returns @n amount of data in form of a TimeSeries tuple, where the first
        element of the tuple is the time vector. If @n == 0 all data is returned,
        if n < 0 a ValueError exception is raised.'''
        logging.debug('Entering get_timeseries.')
        if self.has_data():
            data = self.get_data(n)
            logging.debug('Leaving get_timeseries: %s', data)
            if data.ndim == 1:
                logging.debug('Array is 1-D: %s', data)
                # When the array only has one sample, it is stored as a 1-D array,
                # thus, the cases when ndim != 1 are not valid and raise index error.
                return data[0], data[1:]
            else:
                # Array is multidimensional.
                logging.debug('Data: %s', data)
                return data[:,0], data[:,1:]
        else:
            logging.debug('Leaving get_timeseries: %s, %s', [], [])
            return None, None
        
class RealTimePlot(object):

    def __init__(self, signal, title = None, xlabel = None, ylabel = None, legend = None, interval = 50, blit = True, xlim = 3., ylim = [0., 1.], auto_scroll = True):
        self.fig, self.ax = create_axes()
        self.signal = signal
        self.lines = []
        self._data = np.sin(np.arange(0, 2 * math.pi, 1. / interval))
        if title is not None:
            self.ax.set_title(title)
        if xlabel is not None:
            self.ax.set_xlabel(xlabel)
        if ylabel is not None:
            self.ax.set_ylabel(ylabel)
        if legend is not None:
            for l in legend:
                self.lines.append(self.ax.plot([], [], label = l)[0])
        self.xlim = xlim
        self.ax.set_ylim(ylim)
        self.ax.set_xlim([0, xlim])
        self.auto_scroll = auto_scroll
        plt.legend()
        self.animation = animation.FuncAnimation(self.fig, self.update, interval = interval, blit = blit)      
            
    def _handle_auto_scroll(self, t):
        '''If autoscroll is specified, this function handles it. This should
        not be used by application code.'''
        logging.debug('Current time: %s', t)
        if self.auto_scroll:
            logging.debug('Autoscrolling!!!')
            
            # numpy empty arrays have dimension 0!!!!
            # so we have to use size to take care of this.
            # Empty arrays have size 0.
            if t.ndim == 0 and t.size != 0:
                logging.debug('Time is scalar: %s', t)
                if t > self.xlim:
                    self.ax.set_xlim([t - self.xlim, t])
            elif t.ndim > 0:
                logging.debug('Time is vector: %s', t)
                if t[-1] > self.xlim:    
                    self.ax.set_xlim([t[-1] - self.xlim, t[-1]])
            else:
                logging.warn('Time vector is unknown: %s', t)
                       
                                        
    def update(self, i):
        '''Gets called to redraw the plot. Should not be used by application
        code.'''
        logging.debug('Entering update callback')
        for l in self.lines:
            cx, cy = l.get_data()
            logging.debug('Current line data: %s, %s', cx, cy)
            x, y = self.signal.get_timeseries()
            if x is not None and x is not None:
                logging.debug('Signal time: %s', x)
                l.set_data(np.append(cx, x), np.append(cy, y))
                self._handle_auto_scroll(x)
        self.ax.figure.canvas.draw()
        logging.debug('Leaving update callback')
        return self.lines
    
if __name__ == '__main__':
    s = Signal()
    r = RealTimePlot(s, title = 'Title', xlabel = 'xlabel', ylabel = 'ylabel', legend = ['signal'], blit = False, interval = 100)
    random_data(s, 0.)    
    plt.show()
