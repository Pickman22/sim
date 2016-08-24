import numpy as np
import numpy.polynomial.polynomial as polynomial
import matplotlib.pyplot as plt
import math
import logging
import motor

logging.basicConfig(level=logging.DEBUG)
logging.getLogger().setLevel(logging.DEBUG)


def _compute_gains(K, tau, Ts, Pos):
    z = abs(math.log(Pos)) / math.sqrt((math.log(Pos)) ** 2 + math.pi ** 2)
    wn = 4 / (z * Ts)
    logging.debug('Damping ratio: %s, Natural frequency: %s rads/s', z, wn)
    k1 = (2 * z * wn * tau) / K  # Zero-order coefficient.
    k2 = (tau * wn ** 2 - 1) / K  # First-order coefficient.
    if -z * wn >= 0.:
        logging.warn('Unstable system!!!')
    logging.debug('Stability term: %s', -z * wn)
    logging.debug('Controller gains k1: %s, k2: %s', k1, k2)
    return k1, k2


def _motor_controller(Ts, Pos, K=None, tau=None, R=None,
                      L=None, Kt=None, Kb=None, Kf=None, J=None):
    if K is None or tau is None:
        if R is None or L is None or Kt is None or Kb is None or Kf is None or
        J is None:
            raise ValueError('If K AND tau are not provided, then all motor \
                             parameters are needed!')
            return None
        else:
            K, tau = compute_K_and_tau(R, L, J, Kb, Kt, Kf)
    return _compute_gains(K, tau, Ts, Pos)


def velocity_controller(Ts, Pos, K=None, tau=None, R=None, L=None,
                        J=None, Kt=None, Kb=None, Kf=None):
    ''' Computes PID velocity controller given motor parameters '''
    kp, ki = _motor_controller(Ts, Pos, K, tau, R, L, Kt, Kb, Kf, J)
    return PID_Controller(kp=kp, ki=ki, kd=0.)


def position_controller(Ts, Pos, K=None, tau=None, R=None, L=None,
                        J=None, Kt=None, Kb=None, Kf=None):
    ''' Computes PID position controller given motor parameters '''
    kd, kp = _motor_controller(Ts, Pos, K, tau, R, L, Kt, Kb, Kf, J)
    return PID_Controller(kp=kp, kd=kd, ki=0.)


def motor_velocity_controller(Ts, Pos, m):
    ''' Computes PID velocity controller given motor model '''
    return velocity_controller(Ts, Pos, m.R, m.L, m.J, m.Kt, m.Kb, m.Kf)


def motor_position_contrlller(Ts, Pos, m):
    ''' Computes PID position controller given motor model '''
    return position_controller(Ts, Pos, m.R, m.L, m.J, m.Kt, m.Kb, m.Kf)


def compute_K_and_tau(R, L, J, Kt, Kb, Kf):
    if R <= 10 * L:
        logging.warn('This method only holds for R >> L!')
    K = Kt / (R * J)
    tau = R * J / (Kt * Kb + R * Kf)
    logging.debug('K: %s, tau: %s', K, tau)
    return K, tau


class PID_Controller(object):

    def __init__(self, kp=0., kd=0., ki=0., target=0., I_MAX=None, I_MIN=None,
                 ts=20e-3):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.ts = ts
        self.target = target
        self.error = 0.
        self._ie = 0.
        self._de = 0.
        self._preverr = 0.
        self.I_MIN = I_MIN
        self.I_MAX = I_MAX

    def reset(self):
        self._ie = 0.

    def set_gains(self, kp=None, kd=None, ki=None):
        if kp is not None:
            self.kp = kp
        if kd is not None:
            self.kd = kd
        if ki is not None:
            self.ki = ki

    def _integral(self):
        self._ie += self.error * self.ts
        if self.I_MAX is not None:
            if self._ie > self.I_MAX:
                self._ie = self.I_MAX
        if self.I_MIN is not None:
            if self._ie < self.I_MIN:
                self._ie = self.I_MIN
        return self._ie

    def _derivative(self):
        de = (self.error - self._preverr) / self.ts
        self._preverr = self.error
        return de

    def control(self, y, target=None):
        if target is not None:
            self.target = target
        self.error = self.target - y
        return self.kp * self.error + self.kd * self._derivative() +
        self.ki * self._integral()


if __name__ == '__main__':

    def position_pid(x, t, *args):
        pid, = args
        return pid.control(x[0])  # Uses position as feedback.

    def velocity_pid(x, t, *args):
        pid, = args
        return pid.control(x[1])  # Uses velocity as feedback.

    ti = 0.
    tf = 5.
    ts = 0.001
    t = ti

    Ts = 2  # Settiling time.
    Pos = 0.05  # Maximum overshoot of 5%.

    R = 1.  # Armature resistance.
    Kf = 0.1  # Friction coefficient.
    Kb = 0.01  # Back-emf constant.
    L = 0.05  # Armature inductance.
    Kt = 0.01  # Torque constant.
    J = 0.1  # Rotor's moment of inertia.

    m = motor.DCMotor(ts, R, L, J, Kt, Kf, Kb)
    vel_pid = velocity_controller(Ts, Pos, Kf=Kf, Kb=Kb, L=L, R=R, J=J, Kt=Kt)
    pos_pid = position_controller(Ts, Pos, Kf=Kf, Kb=Kb, L=L, R=R, J=J, Kt=Kt)
    pos_pid.ts = ts
    vel_pid.ts = ts
    pos_pid.target = 1.
    vel_pid.target = 1.

    pt, px, pv = motor.motor_sim(m, ti, tf, ts, pos_pid, on_input=position_pid)
    vt, vx, vv = motor.motor_sim(m, ti, tf, ts, vel_pid, on_input=velocity_pid)

    fig, ax = plt.subplots(2, 2)

    ax[0, 0].plot(pt, px[:, 0:2])
    ax[0, 0].legend(['Position', 'Velocity'])
    ax[0, 0].set_title('Position Controller')
    ax[1, 0].plot(pt, pv, label='Voltage')
    ax[1, 0].set_title('Control Signal')
    ax[1, 0].legend()

    ax[0, 1].plot(vt, vx[:, 1], label='Velocity')
    ax[0, 1].set_title('Velocity Controller')
    ax[0, 1].legend()
    ax[1, 1].plot(vt, vv, label='Voltage')
    ax[1, 1].set_title('Control Signal')
    ax[1, 1].legend()
    plt.show()
