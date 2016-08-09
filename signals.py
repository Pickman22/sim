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

matplotlib.use('TkAgg') # <-- THIS MAKES IT FAST!

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
        logging.info('Initializing Signal class')
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

        
class SerialSignal(Signal):

    def __init__(self, labels, port = '/dev/ttyACM0', baudrate = 9600, timeout = 1., rate = 10.):
        Signal.__init__(self)
        self.port = serial.Serial(port, baudrate, timeout = timeout)
        if not self.port.is_open:
            self.port.open()
        self._stop = True
        self.rate = rate
        self.names = labels
        self.now = time.time()
            
    def _read_data(self):
        raw_data = self.port.readline()
        try:
            json_data = json.loads(raw_data)
            #logging.debug(data)
            #for signal_name, signal in self._signals_dict.iteritems():
            data = [json_data['time']]
            for name in self.names:
                if json_data.has_key(name):
                    data.append(json_data[name])
                else:
                    data.append(0.)
            self.add_data(np.array(data))
            logging.info('Data: %s', data)
        except ValueError:
            logging.error('Could not read data!: %s', raw_data)
        if not self._stop:
            threading.Timer(self.rate / 1000., self._read_data).start()

    def start(self):
        if not self.port.is_open:
            self.port.open()
        self._stop = False
        self._read_data()
        
    def stop(self):
        self._stop = True
        self.port.close()
        
    def get_names(self):
        return self.names
            
class RealTimePlot(object):

    def __init__(self, source, title = None, xlabel = None, ylabel = None,
    interval = 60, blit = True, xlim = 3., ylim = [-2., 2.],
    autoscroll = True, autosize = True, keep_data = False):
        self.fig, self.ax = create_axes()
        self.source = source
        # A source is a stream that has a data array linked to a label name.
        # this is done to link json data obtained through serial port to
        # data that will be plotted, so to avoid mixing values.
        self.lines = [plt.plot([], [], lw = 1.5, label = n)[0] for n in self.source.get_names()]
        self._current_max, self._current_min = 0., 0.
        if title is not None:
            self.ax.set_title(title)
        if xlabel is not None:
            self.ax.set_xlabel(xlabel)
        if ylabel is not None:
            self.ax.set_ylabel(ylabel)
        self.xlim = xlim
        self.ax.set_ylim(ylim)
        self.ax.set_xlim([0, xlim])
        self.autoscroll = autoscroll
        self.autosize = autosize
        self.keep_Data = keep_data
        plt.legend()
        self.animation = animation.FuncAnimation(self.fig, self.update, interval = interval, blit = blit)
        #plt.ion()
        plt.show()
            
            
    def _init_plot(self):
        pass
        
            
    def _handle_autoscroll(self, t):
        '''If autoscroll is specified, this function handles it. This should
        not be used by application code.'''
        logging.debug('Current time: %s', t)
        if self.autoscroll:
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
                logging.warn('Time vector is unknown type: %s', type(t))
               
    def _handle_autosize(self, t):
        pass
     
    def update(self, i):
        '''Gets called to redraw the plot. Should not be used by application
        code.'''
        logging.debug('Entering update callback')
        if self.source.has_data():
            logging.debug('Signal has data!')
            new_t, new_values = self.source.get_timeseries()
            self._handle_autoscroll(new_t)            
            for line, new_value in zip(self.lines, new_values.T):                
                current_t, current_values = line.get_data()
                new_line_values = np.append(current_values, new_value)
                new_line_values
                line.set_data(np.append(current_t, new_t), new_line_values)
        else:
            logging.warn('Real time plot update has no data!')
        #self.fig.canvas.update()
        #self.ax.figure.canvas.draw()
        plt.ion()
        logging.debug('Leaving update callback')
        return self.lines
    
if __name__ == '__main__':
    s = SerialSignal(['x'], baudrate = 115200)
    s.start()
    r = RealTimePlot(s)
    while True:
        try:
            pass
            #time.sleep(1.)
        except KeyboardInterrupt:
            s.stop()
            break
