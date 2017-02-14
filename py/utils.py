import numpy as np
import matplotlib.pyplot as plt
import logging

logging.basicConfig(level=logging.DEBUG)
#logging.getLogger().setLevel(logging.ERROR)


def create_message(data):
    ''' Takes a list @data with bytes to be send via a serial or TCP transport,
    creates a message with the following format:

        START | MSB_SZ | LSB_SZ | DATA | END,

    where START indicates the beginning of a message. MSB_SZ is the most
    significant byte of the payload size, which is 16-bit. LSB_SZ is the less
    significant byte of the payload size. DATA is the payload. And END denotes
    the end of the message. '''

    START, END = 0x00A0, 0x000A

    msg = bytearray(data)
    sz = len(msg)
    if sz <= 0x00 or sz > 0xFFFF:
        raise ValueError('Data list size must be greater than 0 and smaller than 0xFFFF.')
    msg.insert(0, sz & 0x00FF)
    msg.insert(0, (sz >> 8) & 0x00FF)
    msg.insert(0, START)
    msg.append(END)
    return msg

def create_axes(title=None, xlabel=None, ylabel=None, legend=None):
    '''Create a figure with modified parameters so that it looks more
    aesthetic. @title is the figure's title, @xlabel is the x-axis' label,
    @ylabel, the y-axis' label and @legend holds the lines labels.'''
    fig = plt.figure()
    ax = plt.gca()
    if title:
        ax.set_title(title)
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    if legend:
        ax.legend(legend, loc='best', frameon=False)
    ax.spines['bottom'].set_linewidth(2)
    ax.spines['left'].set_linewidth(2)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')
    ax.tick_params(labelsize=14)
    ax.yaxis.grid(True, which='major')
    ax.xaxis.grid(True, which='major')
    return fig, ax

def step(dx, x, ts):
    ''' Euler integration algorithm. @dx is a numpy.ndarray vector that holds
    the derivative of the system. @x is a numpy.ndarray vector that holds the
    current state. @ts is the time step. '''
    if ts <= 0.:
        raise ValueError('Timestep must be positive')
        return np.zeros(x.shape)
    try:
        return dx * ts + x
    except:
        raise ValueError('System dimension mismatch')
        return np.zeros(x.shape)


def append_to_array(array, data):
    return np.hstack((array, data))

def remove_from_list(data, keep_data):
    '''Remove elements from list @data so that the length matches @keep_data'''
    n = len(data) - keep_data
    if n <= 0:
        return data
    else:
        logging.debug('Removing %s from list', n)
        return data[:n]


def remove_from_ndarray(data, keep_data):
    '''Remove elements from @data ndarray so that the length matches @keep_data
    '''
    n = data.size - keep_data
    if n <= 0:
        return data
    else:
        logging.debug('Removing %s from ndarray', n)
    return np.delete(data, slice(0, n))
