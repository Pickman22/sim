import numpy as np
import numpy.polynomial.polynomial as polynomial
import matplotlib.pyplot as plt
import math
import logging
import motor
import abc
import transport, utils
#import msgparser
import struct
import binascii

logging.basicConfig(level=logging.DEBUG)
logging.getLogger().setLevel(logging.DEBUG)

def _compute_gains(K, tau, Ts, Pos):
    z = abs(math.log(Pos)) / math.sqrt((math.log(Pos)) ** 2 + math.pi ** 2)
    wn = 4 / (z * Ts)
    #logging.debug('Damping ratio: %s, Natural frequency: %s rads/s', z, wn)
    k1 = (2 * z * wn * tau) / K  # Zero-order coefficient.
    k2 = (tau * wn ** 2 - 1) / K  # First-order coefficient.
    if -z * wn >= 0.:
        logging.warn('Unstable system!!!')
    logging.debug('System Response: Natural freq: {}, Damping coefficient: {}'.format(wn, z))
    logging.debug('Design parameters -> Max Overshoot: {}, Settling time: {}'.format(Pos, Ts))
#    logging.debug('Stability term: %s', -z * wn)
#    logging.debug('Controller gains k1: %s, k2: %s', k1, k2)
    return k1, k2


def _motor_controller(Ts, Pos, K=None, tau=None, R=None,
                      L=None, Kt=None, Kb=None, Kf=None, J=None):
    if K is None or tau is None:
        if R is None or L is None or Kt is None or Kb is None or Kf is None or \
           J is None:
            raise ValueError('If K AND tau are not provided, then all motor \
                             parameters are needed!')
            return None
        else:
            K, tau = compute_K_and_tau(R, L, J, Kb, Kt, Kf)
            logging.debug('System Parameters: DC Gain: {}, Time Costant: {}'.format(K, tau))
            #logging.debug('Design parameters -> Max Overshoot: {}, Settling time: {}'.format(Pos, Ts, wn))
    return _compute_gains(K, tau, Ts, Pos)


def velocity_controller(Ts, Pos, K=None, tau=None, R=None, L=None,
                        J=None, Kt=None, Kb=None, Kf=None):
    ''' Computes PID velocity controller given motor parameters '''
    kp, ki = _motor_controller(Ts, Pos, K, tau, R, L, Kt, Kb, Kf, J)
    kd = 0.
    logging.debug('Controller gains -> Kp: {}, Ki: {}, Kd: {}'.format(kp, ki, kd))
    return kp, ki, kd


def position_controller(Ts, Pos, K=None, tau=None, R=None, L=None,
                        J=None, Kt=None, Kb=None, Kf=None):
    ''' Computes PID position controller given motor parameters '''
    kd, kp = _motor_controller(Ts, Pos, K, tau, R, L, Kt, Kb, Kf, J)
    ki = 0.
    logging.debug('Controller gains -> Kp: {}, Ki: {}, Kd: {}'.format(kp, ki, kd))
    return kp, ki, kd


def motor_velocity_controller(Ts, Pos, m):
    ''' Computes PID velocity controller given motor model '''
    return velocity_controller(Ts, Pos, R = m.params['R'], L = m.params['L'],
    J = m.params['J'], Kt = m.params['Kt'], Kb = m.params['Kb'],
    Kf = m.params['Kf'])


def motor_position_controller(Ts, Pos, m):
    ''' Computes PID position controller given motor model '''
    return position_controller(Ts, Pos, R = m.params['R'], L = m.params['L'],
    J = m.params['J'], Kt = m.params['Kt'], Kb = m.params['Kb'],
    Kf = m.params['Kf'])

def compute_K_and_tau(R, L, J, Kt, Kb, Kf):
    if R <= 10 * L:
        logging.warn('This method only holds for R >> L!')
    K = Kt / (R * J)
    tau = R * J / (Kt * Kb + R * Kf)
    logging.debug('K: %s, tau: %s', K, tau)
    return K, tau


class Controller(object):

    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def get_output(self):
        pass

    @abc.abstractmethod
    def get_error(self):
        pass

    @abc.abstractmethod
    def get_feedback(self):
        pass

    @abc.abstractmethod
    def get_target(self):
        pass

    @abc.abstractmethod
    def step(self):
        pass

    @abc.abstractproperty
    def ts(self):
        pass


class PID_Controller(Controller):

    def __init__(self, kp=0., ki=0., kd=0., target=0., I_MAX=None, I_MIN=None,
                 ts = 20e-3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ts = ts
        self.target = target
        self.error = 0.
        self._ie = 0.
        self._de = 0.
        self._preverr = 0.
        self.I_MIN = I_MIN
        self.I_MAX = I_MAX
        self.output = 0.
        self.feedback = 0.

    def reset(self):
        self._ie = 0.

    def ts(self):
        return self.ts

    def set_gains(self, kp=None, kd=None, ki=None):
        if kp is not None:
            self.kp = kp
        if kd is not None:
            self.kd = kd
        if ki is not None:
            self.ki = ki

    def get_output(self):
        return self.output

    def get_error(self):
        return self.error

    def get_feedback(self):
        return self.feedback

    def get_target(self):
        return self.target

    def step(self, y, target = 0.):
        self.feedback = y
        self.target = target

        #logging.debug('Feedback: {}'.format(self.feedback))
        #logging.debug('Target: {}'.format(self.target))
        self.error = self.target - self.feedback

        # Compute Integral.
        self._ie += self.error * self.ts

        # Saturate Integral to avoid windup.
        if self.I_MAX is not None:
            self._ie = np.max((self._ie, self.I_MAX))
        if self.I_MIN is not None:
            self._ie = np.min((self._ie, self.I_MIN))

        # Compute Derivative.
        _de = (self.error - self._preverr) / self.ts
        self._preverr = self.error
        self.output = self.kp * self.error + self.kd * _de + self.ki * self._ie
        #logging.debug('Control contribution-> P: {}, I: {}, D: {}'.format(\
        #    self.kp * self.error, self.kd * _de, self.ki * self._ie))
        return self.output


class Motor_PID_Controller(PID_Controller):

    def step(self, x, sensed_x, observed_x, target = 0.):
        #logging.debug('x: {}'.format(x))
        #logging.debug('sensed_x: {}'.format(sensed_x))
        #logging.debug('observed_x: {}'.format(observed_x))
        assert(type(x) is np.ndarray and x.dtype == np.float64)
        assert(type(sensed_x) is np.ndarray and sensed_x.dtype == np.float64)
        assert(type(observed_x) is np.ndarray and observed_x.dtype == np.float64)
        return PID_Controller.step(self, observed_x[1], target)


class ExternController(Controller):

    GET_ERROR_CMD = 0x13
    GET_OUTPUT_CMD = 0x10
    GET_FEEDBACK_CMD = 0x21
    GET_TARGET_CMD = 0x70

    ''' The step command is implemented as follows:
    Due to the fact that the system may or not have sensors, observers attached,
    an identifier is attached before the data for every variable. This way,
    the communication end can detect the data that is being provided. A step
    command has the following format:

        | STEP_CMD | X1_ID | X1_32 | X2_ID | X2_32 | X3_ID | X3_32 | ...
        | SENS_P_ID | SENS_P_32 | SENS_W_ID | SENS_W_32 | SENS_I_ID | SENS_I_32 | ...
        | OBS_P_ID | OBS_P_32 | OBS_W_ID | OBS_W_32 | OBS_I_ID | OBS_I_32 | ...
        | TIME_ID | TIME_32 |

    If the ID fied for a signal is not present. It means that the
    state / sensor / obsever is not available. Each value is represented as a
    32-bit integer. A scaling of 1e3 is assumed for floating point signals.
    '''

    STEP_CMD = 0x80

    X1_ID = 0x00
    X2_ID = 0x01
    X3_ID = 0x02
    X_ID = [X1_ID, X2_ID, X3_ID]

    SENS_P_ID = 0x10
    SENS_W_ID = 0x11
    SENS_I_ID = 0x12
    SENS_ID = [SENS_P_ID, SENS_W_ID, SENS_I_ID]

    OBS_P_ID = 0x20
    OBS_W_ID = 0x21
    OBS_I_ID = 0x22
    OBS_ID = [OBS_P_ID, OBS_W_ID, OBS_I_ID]

    TIME_ID = 0x30
    TARGET_ID = 0x31

    def __init__(self, transport, ts, scaling = 1e3, bytes = 4):
        self.transport = transport
        self.scaling = scaling
        self._max_data_value = 2 ** int(8 * bytes)
        self._max_data_bytes = int(bytes)
        self.ts = ts

    def _check_connection(self):
        if not self.transport.is_connected:
            raise RuntimeError('Connection lost!')

    def ts(self):
        return self.ts

    def _write_get_cmd(self, cmd):
        self._check_connection()
        cmd = struct.pack('>B', cmd)
        self.transport.write(cmd)
        return struct.unpack('>i', self.transport.read())[0] / self.scaling

    def get_output(self):
        return self._write_get_cmd(self.GET_OUTPUT_CMD)

    def get_error(self):
        return self._write_get_cmd(self.GET_ERROR_CMD)

    def get_feedback(self):
        return self._write_get_cmd(self.GET_FEEDBACK_CMD)

    def get_target(self):
        return self._write_get_cmd(self.GET_TARGET_CMD)

    def step(self, x, sensed_x, observed_x, target = 0.):
        self._check_connection()
        self._check_overflow((x, sensed_x, observed_x, target))
        cmd = struct.pack('>B', self.STEP_CMD)
        xbytes = self._to_bytes(x, self.X_ID)
        sensed_x_bytes = self._to_bytes(sensed_x, self.SENS_ID)
        observed_x_bytes = self._to_bytes(observed_x, self.OBS_ID)
        target_bytes = self._to_bytes(target, self.TARGET_ID)
        self.transport.write(cmd + xbytes + sensed_x_bytes + observed_x_bytes +\
                             target_bytes)

        # Wait for response. The response must be unpackable to 32-bit int.
        return struct.unpack('>i', self.transport.read())[0] / self.scaling

    def _check_overflow(self, values):
        # Runtime warning is raised in this function due to values containing
        # np.nan. The greater than evaluates to False, which is fine for this
        # purpose.
        for value in values: # Iterate over signal groups: states, sensors, etc.
            assert(type(value) is np.ndarray or type(value) is np.float64 or type(value) is float)
            if value is np.ndarray:
                value = value[~np.isnan(value)]
            logging.debug('Check overflow on: {}'.format(value))
            if ((self.scaling * value).astype(int) > self._max_data_value).any():
                raise RuntimeError('Detected controller data overflow: {},\
                max data value: {}'.format((self.scaling * value).astype(int),\
                self._max_data_value))

    def _to_bytes(self, var, id):
        assert(type(var) is np.ndarray or type(var) is np.float64 or type(var) is float)
        if type(var) is np.ndarray:
            #x = (var * self.scaling).astype(int)
            xbytes = ''
            for _id, _x in zip(id, var):
                if not np.isnan(_x):
                    _x = int(_x * self.scaling)
                    xbytes += struct.pack('>B', _id) + struct.pack('>i', _x)
                else:
                    # NAN means that the signal is not available. We are safe
                    # to ignore this condition.
                    pass
        elif (type(var) is np.float64 or type(var) is float) and not np.isnan(var):
            xbytes = struct.pack('>B', id) + struct.pack('>i', int(var * self.scaling))

        else:
            raise RuntimeError('Type not supporrted!')
        logging.debug('Unpacked signal: {}'.format(var))
        logging.debug('Packed signal: {}'.format(binascii.hexlify(xbytes)))
        return xbytes
