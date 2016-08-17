import numpy as np
import numpy.polynomial.polynomial as polynomial
import matplotlib.pyplot as plt
import math
import logging
import motor

logging.basicConfig(level = logging.DEBUG)
logging.getLogger().setLevel(logging.DEBUG)

def _vel_pid_gains(K, tau, Ts, Pos):
    z = abs(math.log(Pos)) / math.sqrt((math.log(Pos)) **2 + math.pi ** 2)
    wn = 4 / (z * Ts)
    logging.debug('damping: %s, natural freq: %s', z, wn)
    kp = (2 * z * wn * tau) / K
    ki = (tau * wn ** 2 - 1) / K
    logging.debug('Real pole: %s', -z * wn)
    logging.debug('Controller gains kp: %s, ki: %s', kp, ki)
    return kp, 0., ki

def _pos_pid_gains(K, tau, Ts, Pos):
    z = abs(math.log(Pos)) / math.sqrt((math.log(Pos)) **2 + math.pi ** 2)
    wn = 4 / (z * Ts)
    logging.debug('damping: %s, natural freq: %s', z, wn)
    p1 = wn * (-z + np.sqrt(z**2 - 1 + 0j))
    p2 = wn * (-z - np.sqrt(z**2 - 1 + 0j))
    p3 = -10 * z * wn
    logging.debug('Poles %s', (p1, p2, p3))
    poly = polynomial.polyfromroots((p1, p2, p3))
    logging.debug('Polynomial: %s', poly)
    #poly = poly / poly[-1] # Make sure highest order coefficient is equals 1.
    if poly.imag.any() > 1e-3:
        logging.warn('Characteristic polynomial has complex coefficients: %s', poly.imag)
    kp = (tau * poly[1].real - 1.) / K
    kd = tau * poly[2].real / K
    ki = tau * poly[0].real / K
    logging.debug('Controller gains: %s, %s, %s', kp, kd, ki)
    return (kp, kd, ki)

def _motor_controller(Ts, Pos, _pid_gains_func, K = None, tau = None, R = None, L = None, 
    Kt = None, Kb = None, Kf = None, J = None, pid = None):
    if K is None or tau is None:
        if R is None or L is None or Kt is None or Kb is None or Kf is None or J is None:
            raise ValueError('If K and tau are not provided, then all motor parameters are needed!')
            return None
        else:
            K, tau = compute_K_and_tau(R, L, Kt, Kb, Kf, J)
    kp, kd, ki = _pid_gains_func(K, tau, Ts, Pos)
    if pid is None:
        pid = PID_Controller(kp, kd, ki)
    else:
        pid.set_gains(kp, kd, ki)
    return pid

def motor_velocity_controller(Ts, Pos, K = None, tau = None, R = None, L = None, 
    Kt = None, Kb = None, Kf = None, J = None, pid = None):
    pid = _motor_controller(Ts, Pos, _vel_pid_gains, K, tau, R, L, Kt, Kb, Kf, J, pid)
    
def motor_position_controller(Ts, Pos, K = None, tau = None, R = None, L = None, 
    Kt = None, Kb = None, Kf = None, J = None, pid = None):
    pid = _motor_controller(Ts, Pos, _pos_pid_gains, K, tau, R, L, Kt, Kb, Kf, J, pid)

def compute_K_and_tau(R, L, Kt, Kb, Kf, J):
    if not L < 10 * R:
        logging.warn('This method only holds for R >> L!')
    K = Kt / (R* J)
    tau = R * J / (Kt * Kb + R * Kf)
    logging.debug('K: %s, tau: %s', K, tau)
    return K, tau
    
class PID_Controller(object):

    def __init__(self, kp = 0., kd = 0., ki = 0., r = 0., I_MAX = None, I_MIN = None, ts = 20e-3):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.ts = ts
        self.r = r
        self.error = 0.
        self._ie = 0.
        self._de = 0.
        self._preverr = 0.
        #self.I_MIN = I_MIN
        #self.I_MAX = I_MAX

    def reset(self):
        self._ie = 0.
        
    def set_gains(self, kp = None, kd = None, ki = None):
        if kp is not None:
            self.kp = kp
        if kd is not None:
            self.kd = kd
        if ki is not None:
            self.ki = ki
        
    def _integral(self):
        self._ie += self.error * self.ts
        '''
        if self.I_MAX is not None:
            if self._ie > self.I_MAX:
                self._ie = self.I_MAX
        if self.I_MIN is not None:
            if self._ie < self.I_MIN:
                self._ie = self.I_MIN
        '''
        return self._ie
       
    def _derivative(self):
        de = (self.error - self._preverr) / self.ts
        self._preverr = self.error
        return de
        
    def control(self, y, r = None):
        if r is not None:
            self.r = r
        self.error = self.r - y
        return self.kp * self.error + self.kd * self._derivative() + self.ki * self._integral()


ti = 0.
tf = 5.
ts = 0.01
t = ti

pid = PID_Controller(r = 1., ts = ts)

def calc_pid(x, t):
  #return 1.
  return pid.control(x[1])
    
if __name__ == '__main__':

    Ts = 2 # Tiempo de estabilizacion
    Pos = 0.05 # Sobretiro maximo de 5%.
    
    R = 1.
    Kf = 0.1
    Kb = 0.01
    L = 0.05
    Kt = 0.01
    J = 0.1

    m = motor.DCMotor(ts, R = R, Kf = Kf, Kb = Kb, J = J, L = L, Kt = Kt)
    motor_velocity_controller(Ts, Pos, K = 0.1, tau = 0.999, pid = pid)
    #motor_velocity_controller(Ts, Pos, Kf = Kf, Kb = Kb, L = L, R = R, J = J, Kt = Kt, pid = pid)
    mt, mx = motor.motor_sim(m, ti, tf, ts, on_input = calc_pid, plot = True)
           

