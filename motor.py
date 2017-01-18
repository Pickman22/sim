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
from observer import MotorVelocityObserver, MotorSlidingModeObserver

logging.basicConfig(level = logging.DEBUG)

class DCMotor(DynamicSystem):

    _POS_SENSE_IDX = 0
    _VEL_SENSE_IDX = 1
    _CURRENT_SENSE_IDX = 2

    def __init__(self, ts = 1e-3, R=1., L=1., J=1., Kt=1., Kf=1., Kb=1., Ks = 0.,
                 x0=[0., 0., 0], position_sensor = None, velocity_sensor = None,
                 current_sensor = None):

        self.params = {
            'R':  R,  # Armature resistance.
            'Kf': Kf,  # Friction coefficient.
            'Kb': Kb,  # Back-emf constant.
            'L':  L,  # Armature inductance.
            'Kt': Kt,  # Torque constant.
            'J':  J,  # Rotor's moment of inertia.
            'Ks': Ks, # Static friction coefficient.
        }

        self.x = np.zeros([3, 1])
        self.A = np.array([
            [0.,          1.,     0.],
            [0.,    -Kf / J,  Kt / J],
            [0.,    -Kb / L,  -R / L]
        ])
        self.B = np.array([
            [0.],
            [0.],
            [1. / L]
        ])
        self.x = np.array(x0).reshape(3, 1)
        self.dx = np.zeros([3, 1]) #Initialize to zero. It'll be computed later.
        self.u = 0.
        self.ts = ts
        self.dim = self.x.size
        self._sensors = [position_sensor, velocity_sensor, current_sensor]

    def _get_sensor_value(self, idx):
        value = None
        if self._sensors[idx] is None:
            value = None
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

    def dynamics(self, u):
        dx = None
        try:
            self.A.reshape([3, 3])
            self.B.reshape([3, 1])
            self.x.reshape([3, 1])
            dx = self.A.dot(self.x) + u * self.B \
            - np.array([0., self.params['Ks'], 0.]).dot(np.sign(self.x))
        except:
            raise ValueError('System dynamics dimension mismatch')
            return np.zeros([3, 1])
        return dx

class MotorSimulation(object):

    _STATES_IDX = 0
    _SENSOR_IDX = 1
    _OBSERVER_IDX = 2

    def __init__(self, motor, t0, tf, ts, on_input = None, on_output = None,
                 on_step = None, controller = None, observer = None,
                 plot_results = True, target = None):

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

        #self.on_input = on_input
        self.plot_results = plot_results
        self.target = target
        self._current_target = 0.

    def reset_motor(self):
        self.motor.set_states(self._motor_x0)

    def config(self, t0, tf, ts = None):
        if self.ts is not None:
            self.ts = ts
        self.t = np.arange(t0, tf, self.ts)
        if self.t.size == 0:
            raise ValueError('Sampling time must be less than tf - t0.')
            logging.debug('Simulation time step: {0}'.format(self.ts))

    def _check_ts(self, obj):
        if obj.ts < self.ts:
            raise ValueError('Object time step must be smaller than simulation time step.')
        #else:
            #self._obs_counts = round(self.observer.ts / self.ts)
            #logging.debug('Object time step: {0}'.format(self._obs_counts * self.ts))
            #self.log['observer'] = np.zeros([3, self.t.size])
            #self.log['observer'][:, 0] = self.get_observer_estimate().reshape(1, 3)

    def set_observer(self, observer):
        # Determine observer counts needed by each simulation steps counts.
        self.observer = observer
        self._obs_current_counts = 1
        self._check_ts(self.observer)
        #if self.observer is not None:
            #if self.observer.ts < self.ts:
            #    raise ValueError('Observer time step must be smaller than simulation time step.')
            #else:
                #self._obs_counts = round(self.observer.ts / self.ts)
                #logging.debug('Observer time step: {0}'.format(self._obs_counts * self.ts))
                #self.log['observer'] = np.zeros([3, self.t.size])
                #self.log['observer'][:, 0] = self.get_observer_estimate().reshape(1, 3)

    def set_controller(self, controller):
        # Determine controller counts needed for each simulation step counts.
        self.controller = controller
        self._controller_current_counts = 1
        self._check_ts(self.controller)
        #if self.controller is not None:
            #if self.controller.ts < self.ts:
                #raise ValueError('Controller time step must be smaller than simulation time step.')
            #else:
                #self._controller_counts = round(self.controller.ts / self.ts)
                #self.log['control_signal'] = np.zeros(self.t.size)
                #self.log['tracking_error'] = np.zeros(self.t.size)
                #self.log['target'] = np.zeros(self.t.size)
                #self._ref = copy.deepcopy(target)
                #logging.debug('Controller time step: {0}'.format(self._controller_counts * self.ts))
                #logging.debug('Control signal size: {}'.format(self.log.['']))

    def _get_target(self, t):
        ret = 0.
        if self.target is not None:
            ret = self.target(t)
        return ret

    def _step_observer(self, sensed_x):
        observed_x = None
        if self.observer is not None:
            if self._obs_current_counts == self._obs_counts:
                observed_x = self.observer.step(sensed_x)
                self._obs_current_counts = 1
                #self.log['observer'] = utils.append_to_array(self.log['observer'], observed_x)
            else:
                self._obs_current_counts += 1
                observed_x = self.get_observer_estimate()
        return observed_x

    def _step_controller(self, t):
        u = 0.
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
            return self.observer.get_value().reshape(3, 1)

    def _step_simulation(self, t, idx):
        u = self._get_input(t)

        if self.controller is not None:
            self.log['control_signal'][idx] = u
            self.log['tracking_error'][idx] = self.controller.get_error()
            self.log['target'][idx] = self._current_target
        # Perform one integration step and get system states. This are usually
        # not available in real applications, unless there's a sensor in the
        # system.
        x = self.motor.step(u)
        #self.log['states'] = utils.append_to_array(self.log['states'], x)
        self.log['states'][:, idx] = x.reshape(1, 3)

        # Read sensors of the system. If there are no sesors attached, the
        # returned value is None for that measurement. The order of the
        # sensor measurements matches that of the states:
        # position, velocity, current.
        sensed_x = self.motor.get_sensor_data()
        #self.log['sensors'] = utils.append_to_array(self.log['sensors'], sensed_x)
        self.log['sensors'][:, idx] = sensed_x.reshape(1, 3)

        # Perform one integration step for the observer. The observer input
        # is based of the sensors of the system. If this data is None,
        # then the observer is responsible for handling this case.
        observed_x = self._step_observer(sensed_x)
        if self.observer is not None and observed_x is not None:
            self.log['observer'][:, idx] = observed_x.reshape(1, 3)
        # All the observers get all the measurements available for the system.
        # This is make implementation easier. This way, if more observers are
        # added to the system, the observer has access to all the current
        # measurements. If a measurement needed by the observer is not available
        # then the observer is responsible for handling this condition.

    def _get_input(self, t):
        # If it's time to step the controller, do it. Otherwise the counter is
        # incremented and the current output is used again.
        if self.controller is not None:
            self._step_controller(t)
            u = self.controller.get_output()

        # If a controller is not supplied, then the input is taken
        # from supplied function.
        elif self.on_input is not None:
            u = self.on_input(t,
                              self.motor.get_states(),
                              self.motor.get_sensor_data(),
                              self.get_observer_estimate())
        # Otherwise, the input is assumed to always be zero.
        else:
            u = 0.
        return u

    def _plot_results(self):
        plt.figure()
        subplt = [311, 312, 313]
        signal_names = ['Position (rad)', 'Velocity (rad/s)', 'Current (amps)']

        #logging.debug('Time vector size: {0}'.format(self.t.size))
        #logging.debug('State shape: {0}'.format(self.log['states'].shape))
        #logging.debug('Sensor shape: {0}'.format(self.log['sensors'].shape))
        #logging.debug('Observer shape: {0}'.format(self.log['observer'].shape))

        for idx, (x, sensed_x, observed_x) in \
        enumerate(zip(self.log['states'], self.log['sensors'], \
        self.log['observer'])):
            plt.subplot(subplt[idx])
            # All states are always plotted as target.
            plt.plot(self.t, x)
            legends =['State']
            # If the first element of the array is None, it means that there is
            # not a sensor attached to the system to measure this particular state.
            logging.debug('Testing sensor[0]: {}'.format(sensed_x[0]))
            if not np.isnan(sensed_x[0]):
                plt.plot(self.t, sensed_x)
                legends.append('Sensor')
            else:
                logging.debug('Skipping None sensor data {}.'.format(idx))
            # In the same fashion, if the first element of the observer is None,
            # this means that the observer is not estimating this state.
            logging.debug('Testing observer[0]: {}'.format(observed_x[0]))
            if not np.isnan(observed_x[0]):
                t = np.linspace(self.t[0], self.t[-1], observed_x.size)
                plt.step(t, observed_x)
                legends.append('Observer')
            else:
                logging.debug('Skipping None observer data {}'.format(idx))
            plt.legend(legends, loc = 'best')
            plt.ylabel(signal_names[idx])
        plt.xlabel('Time(s)')

        if self.controller is not None:
            #t = np.arange(self.t[0], self.t[-1], self.controller.ts)
            #t = np.linspace(self.t[0], self.t[-1], self.log['control_signal'].size)
            logging.debug('Time vector size: {}'.format(t.size))
            logging.debug('Reference size: {}'.format(self.log['target'].size))
            logging.debug('Control size: {}'.format(self.log['control_signal'].size))
            logging.debug('Tracking error size: {}'.format(self.log['tracking_error'].size))
            plt.figure()
            plt.subplot(211)
            plt.step(self.t, self.log['control_signal'])
            plt.ylabel('Control Signal (Volts)')
            plt.subplot(212)
            plt.step(self.t, self.log['tracking_error'])
            plt.step(self.t, self.log['target'])
            plt.ylabel('Tracking Error')
            plt.legend(['Tracking Error', 'Reference'], loc = 'best')
        plt.xlabel('Time(s)')

    def _init_log(self):
        self.log = {'states': np.zeros([3, self.t.size]), #self.motor.get_states(),
                    'control_signal': None,
                    'sensors': np.zeros([3, self.t.size]),
                    'observer': None,
                    'tracking_error': None,
                    'target': None}
        # Initialize State Log.
        self.log['states'][:, 0] = self.motor.get_states().reshape(1, 3)
        # Initialize Controller Log.
        if self.controller is not None:
            self.log['control_signal'] = np.zeros(self.t.size)
            self.log['tracking_error'] = np.zeros(self.t.size)
            self.log['target'] = np.zeros(self.t.size)
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

    def run(self):

        self._init_log()

        self._obs_counts = round(self.observer.ts / self.ts)
        self._controller_counts = round(self.controller.ts / self.ts)

        _print_count = 50
        now = time.time()
        # The simulation is not run for the first time sample, for this is the
        # initial condition, which is already supplied by the user.
        for idx, t in enumerate(self.t[1:]):
            # This is the simulation loop. It will run as long as specified.
            # Get motor input.
            # Integrate motor dynamics one step.
            self._step_simulation(t, idx + 1)
            if idx > _print_count:
                print 'Simulation time: ', t, 's\r',
                _print_count += _print_count

        print('Simulation fininshed. It took: {0} seconds'.format(time.time() - now))

        if self.plot_results:
            self._plot_results()
            #plt.show()
        return sim.log

if __name__ == '__main__':

    sim_ts = 1e-3
    t0 = 0.
    tf = 4.
    ctrl_ts = 0.02
    obs_ts = sim_ts

    m = DCMotor(R = 0.19, L = 0.0005, J = 7.5e-5, Kf = 2e-5, Kb =0.0323,
                Kt = 0.0323, position_sensor = Encoder(100))

    kp, kd, ki = controller.motor_position_controller(0.05, 0.5, m)

    position_controller = controller.Motor_PID_Controller(2. * kp, kd, 0.5, ts = ctrl_ts)

    #target = 10 * np.sin(2 * np.arange(t0, tf, ctrl_ts))

    def target(t):
        #logging.debug('Calling get target: {}'.format(t))
        return 5 * np.sin(3 * t)

    sim = MotorSimulation(m, t0, tf, sim_ts, target = target,
                          observer = MotorSlidingModeObserver(20, obs_ts, tau = 0.03),
                          #observer = MotorVelocityObserver(obs_ts, m.get_states()),
                          controller = position_controller, plot_results = True)
    sim.run()

    #sim.set_observer(MotorVelocityObserver(obs_ts, m.get_states()))
    #sim.reset_motor()

    #sim.run()
    plt.show()
    #print sim.log
