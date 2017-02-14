import numpy as np
import logging
import abc
import copy
import time
import matplotlib.pyplot as plt
import utils
import controller
from dynamics import DynamicSystem
from sensor import Encoder
from observer import MotorPositionDifferentiator, MotorSlidingModeObserver
import transport

logging.basicConfig(level = logging.DEBUG)

class DCMotor(DynamicSystem):

    _POS_SENSE_IDX = 0
    _VEL_SENSE_IDX = 1
    _CURRENT_SENSE_IDX = 2

    def __init__(self, ts=1e-3, R=1., L=1., J=1., Kt=1., Kf=1., Kb=1.,
                 x0=[0., 0., 0], position_sensor = None, velocity_sensor = None,
                 current_sensor = None):

        self.params = {
            'R':  R,  # Armature resistance.
            'Kf': Kf,  # Friction coefficient.
            'Kb': Kb,  # Back-emf constant.
            'L':  L,  # Armature inductance.
            'Kt': Kt,  # Torque constant.
            'J':  J,  # Rotor's moment of inertia.
        }

        self._reduced_order = False

        for k, v in self.params.iteritems():
            if v < 0.:
                raise ValueError('{} must be greater than zero.'.format(k))

        self.x = np.zeros([3, 1])

        if self.params['L'] > 0.:
            logging.debug('Using motor second order model.')
            self.A = np.array([[0.,          1.,     0.],
                               [0.,    -Kf / J,  Kt / J],
                               [0.,    -Kb / L,  -R / L]])

            self.B = np.array([[0.],
                               [0.],
                               [1. / L]])
        else:
            self._reduced_order = True
            logging.debug('Using reduced order model.')
            self.A = - self.params['Kf'] / self.params['J']
            self.B = self.params['Kt'] / self.params['J']

        self.x = np.array(x0).reshape(3, 1)
        self.dx = np.zeros([3, 1]) #Initialize to zero. It'll be computed later.
        self.u = 0.
        self.ts = ts
        self.dim = self.x.size
        self._sensors = [position_sensor, velocity_sensor, current_sensor]

    def _get_sensor_value(self, idx):
        value = np.nan
        if self._sensors[idx] is None:
            value = np.nan
        else:
            value = self._sensors[idx].get_value(self.x[idx])
        return value

    def get_states(self):
        return copy.deepcopy(m.x).reshape([3, 1])

    def set_states(self, x):
        if x.shape != (3, 1):
            raise ValueError('Invalid state dimentions.')
        else:
            self.x = copy.deepcopy(x).reshape([3, 1])

    def get_position(self):
        return self._get_sensor_value(self._POS_SENSE_IDX)

    def get_velocity(self):
        return self._get_sensor_value(self._VEL_SENSE_IDX)

    def get_current(self):
        return self._get_sensor_value(self._CURRENT_SENSE_IDX)

    def get_sensor_data(self):
        return np.array([self.get_position(), self.get_velocity(), \
                        self.get_current()]).reshape(3, 1)

    def _reduced_order_dynamics(self, u):
        self.x[2] = (u - self.params['Kb'] * self.x[1]) / self.params['R']
        self.dx[0] = self.x[1]
        self.dx[1] = -self.A * self.x[1] + self.B * self.x[2]
        self.dx[2] = 0.
        return self.dx

    def _dynamics(self, u):
        try:
            self.A.reshape([3, 3])
            self.B.reshape([3, 1])
            self.x.reshape([3, 1])
            dx = self.A.dot(self.x) + u * self.B
        except Exception:
            raise ValueError('System dynamics dimension mismatch')
            return np.zeros([3, 1])
        return dx

    def dynamics(self, u):
        if self._reduced_order == True:
            self.dx = self._reduced_order_dynamics(u)
        else:
            self.dx = self._dynamics(u)
        return self.dx

class MotorSimulation(object):

    _STATES_IDX = 0
    _SENSOR_IDX = 1
    _OBSERVER_IDX = 2

    def __init__(self, motor, t0, tf, ts, controller = None, observer = None,
                 plot_results = True, target = False):

        if (t0 >= tf):
            raise ValueError('tf: {0} must be greater than t0: {1}'.format(tf, t0))

        if ts <= 0:
            raise ValueError('ts" {0} must be positive.'.format(ts))

        self.ts = ts
        self.motor = motor
        self.motor.ts = ts
        self._motor_x0 = self.motor.get_states()
        self.config(t0, tf, ts)

        self.set_observer(observer)
        self.set_controller(controller)

        self.plot_results = plot_results
        self.target = target

    def reset_motor(self):
        self.motor.set_states(self._motor_x0)

    def config(self, t0, tf, ts = None):
        if self.ts is not None and self.ts <= 0.:
            raise ValueError('Simulation timestep must be positive.')
        elif self.ts is not None and self.ts > 0.:
            self.ts = ts
        if tf <= t0:
            raise ValueError('Simulation tf must be greater than t0.')
        self.t = np.arange(t0, tf, self.ts)
        if self.t.size == 0:
            raise ValueError('Time vector size cannot be 0')

    def set_transport(self, transport):
        if transport is not None:
            self.transport = transport
            if not self.is_connected:
                self.transport.connect()

    def set_observer(self, observer):
        self.observer = observer

    def set_controller(self, controller):
        self.controller = controller

    def _get_target(self, t):
        ret = 0.
        if self.target is not None:
            ret = self.target(t)
        return ret

    def _step_observer(self, sensed_x):
        observed_x = 0.
        if self.observer is not None:
            if self._obs_current_counts == self._obs_counts:
                observed_x = self.observer.step(sensed_x)
                self._obs_current_counts = 1
            else:
                self._obs_current_counts += 1
                observed_x = self.get_observer_estimate()
        return observed_x

    def _step_controller(self, t):
        u = 0.
        if self.controller:
            if self._controller_current_counts == self._controller_counts:
                self._current_target = self._get_target(t)
                u = self.controller.step(self.motor.get_states(),
                                     self.motor.get_sensor_data(),
                                     self.get_observer_estimate(),
                                     target = self._get_target(t))
                self._controller_current_counts = 1
            else:
                self._controller_current_counts += 1
                u = self.controller.get_output()
        return u

    def get_observer_estimate(self):
        if self.observer is None:
            return None
        else:
            return self.observer.get_estimate().reshape(3, 1)

    def _step_simulation(self, t, idx):
        u = self._step_controller(t)
        #logging.debug('Controller output: {}'.format(u))
        if self.controller is not None:
            self.log['control_signal'][idx] = self.controller.get_output()
            self.log['tracking_error'][idx] = self.controller.get_error()
            self.log['target'][idx] = self.controller.get_target()
            self.log['feedback'][idx] = self.controller.get_feedback()
        # Perform one integration step and get system states. This are usually
        # not available in real applications, unless there's a sensor in the
        # system.
        x = self.motor.step(u)
        self.log['states'][:, idx] = x.reshape(1, 3)

        # Read sensors of the system. If there are no sesors attached, the
        # returned value is None for that measurement. The order of the
        # sensor measurements matches that of the states:
        # position, velocity, current.
        sensed_x = self.motor.get_sensor_data()
        self.log['sensors'][:, idx] = sensed_x.reshape(1, 3)

        # Perform one integration step for the observer. The observer input
        # is based of the sensors of the system. If this data is None,
        # then the observer is responsible for handling this case.
        # All the observers get all the measurements available for the system.
        # This is make implementation easier. This way, if more observers are
        # added to the system, the observer has access to all the current
        # measurements. If a measurement needed by the observer is not available
        # then the observer is responsible for handling this condition.
        observed_x = self._step_observer(sensed_x)
        if self.observer is not None and observed_x is not None:
            self.log['observer'][:, idx] = observed_x.reshape(1, 3)

    def _plot_results(self):
        plt.figure()

        for splt, x, sensx, obsx, _ylabel in zip([311, 312, 313],\
                                                 self.log['states'],\
                                                 self.log['sensors'],\
                                                 self.log['observer'],\
                                                 ['Position (rad)', 'Velocity (rad/s)', 'Current (Amps)']):

            plt.subplot(splt)
            _legend = ['State']
            if np.isnan(obsx).any():
                pass
            else:
                plt.step(self.t, obsx)
                _legend.append('Observer')
            if np.isnan(sensx).any():
                pass
            else:
                plt.step(self.t, sensx)
                _legend.append('Sensor')
            plt.plot(self.t, x)
            plt.legend(_legend, loc = 'best')
            plt.ylabel(_ylabel)
        plt.xlabel('Time (s)')

        if self.controller is not None:

            plt.figure()

            plt.subplot(311)
            plt.step(self.t, self.log['control_signal'])
            plt.ylabel('Control Signal (Volts)')

            plt.subplot(312)
            plt.step(self.t, self.log['feedback'])
            #plt.step(self.t, self.log['states'][1, :])
            plt.step(self.t, self.log['target'])
            plt.ylabel('Performance')
            plt.legend(['Feedback', 'Target'], loc = 'best')

            plt.subplot(313)
            plt.step(self.t, self.log['tracking_error'])
            plt.ylabel('Error')

        plt.xlabel('Time(s)')

    def _init_log(self):

        self.log = {'states': np.zeros([3, self.t.size]),
                    'control_signal': np.nan,
                    'sensors': np.array([np.nan, np.nan, np.nan]),
                    'observer': np.array([np.nan, np.nan, np.nan]),
                    'tracking_error': None,
                    'target': np.nan,
                    'feedback': np.nan,
                    }

        # Initialize State Log.
        self.log['states'][:, 0] = self.motor.get_states().reshape(1, 3)
        # Initialize Controller Log.
        if self.controller is not None:
            self.log['control_signal'] = np.zeros(self.t.size)
            self.log['tracking_error'] = np.zeros(self.t.size)
            self.log['target'] = np.zeros(self.t.size)
            self.log['feedback'] = np.zeros(self.t.size)
        # Initialize Controller Log.
        if self.observer is not None:
            self.log['observer'] = np.zeros([3, self.t.size])
            self.log['observer'][:, 0] = self.get_observer_estimate().reshape(1, 3)
        # Initialize Sensor Log.
        for data in self.motor.get_sensor_data():
            if data is not None:
                logging.debug('Detected sensors attached to the system.')
                self.log['sensors'] = np.zeros([3, self.t.size])
                self.log['sensors'][:, 0] = self.motor.get_sensor_data().reshape(1, 3)
                break

    def _motor_has_sensors(self):
        ret = False
        for data in self.motor.get_sensor_data():
            if data is not None:
                ret = True
        return ret

    def run(self):
        self._init_log()
        if self.observer:
            if self.observer.ts < self.ts:
                raise ValueError('Observer time step: {} is invalid.'.format(self.observer.ts))
            self._obs_counts = round(self.observer.ts / self.ts)
            self._obs_current_counts = 1

        if self.controller:
            if self.controller.ts < self.ts:
                raise ValueError('Controller time step: {} is invalid'.format(self.controller.ts))
            self._controller_counts = round(self.controller.ts / self.ts)
            self._controller_current_counts = 1

        #_print_count = 50
        now = time.time()
        # The simulation is not run for the first time sample, for this is the
        # initial condition, which is already supplied by the user.
        _sim_progress = 0
        _prev_sim_progress = 0
        for idx, t in enumerate(self.t[1:]):
            # This is the simulation loop. It will run as long as specified.
            # Get motor input.
            # Integrate motor dynamics one step.
            self._step_simulation(t, idx + 1)
            _sim_progress = int(round(100 * (t / self.t[-1])))
            if _sim_progress > _prev_sim_progress:
                _prog = int(_sim_progress / 5) * '-' + (20 - int(_sim_progress/ 5)) * ' '
                print 'Simulation progress: [' +  _prog + ']', _sim_progress, '%\r',
                _prev_sim_progress = _sim_progress

        print('\r\nSimulation took: {0:.2f} seconds'.format(time.time() - now))

        if self.plot_results:
            self._plot_results()
        return sim.log

if __name__ == '__main__':
    sim_ts = 1e-3
    t0 = 0.
    tf = 2.
    ctrl_ts = 20e-3
    obs_ts = ctrl_ts

    m = DCMotor(R = 0.19, L = 0.001, J = 7.5e-5, Kf = 2e-5, Kb =0.0323,
                Kt = 0.0323, position_sensor = Encoder(400))

    kp, ki, kd = controller.motor_velocity_controller(0.02, 0.05, m)
    print('Controller gains -> Kp: {}, Ki: {}, kd: {}'.format(kp, ki, kd))
    pid_controller = controller.Motor_PID_Controller(kp, ki, kd, ts = ctrl_ts)

    ext_controller = controller.ExternController(transport.TCPServerTransport(), 20e-3)

    def target(t):
        return 10. * np.sin(2 * np.pi * 2. * t)

    sim = MotorSimulation(m, t0, tf, sim_ts, target = target,
                          #observer = MotorSlidingModeObserver(20, obs_ts, tau = 0.03),
                          observer = MotorPositionDifferentiator(obs_ts, m.get_states()),
                          controller = ext_controller,
                          plot_results = True)

    sim.run()
    ext_controller.transport.disconnect()
    plt.show()
