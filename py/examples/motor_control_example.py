import signals
import motor
import controller
import numpy as np


class MotorControl(signals.SystemSimulation):
    ''' Defines animation '''

    def init(self):
        params = {
            'R': 1.,  # Armature resistance.
            'Kf': 0.1,  # Friction coefficient.
            'Kb': 0.01,  # Back-emf constant.
            'L': 0.05,  # Armature inductance.
            'Kt': 0.01,  # Torque constant.
            'J': 0.1  # Rotor's moment of inertia.
        }
        Ts = 2  # Settling time.
        Pos = 0.05  # Maximum overshoot

        self.m = motor.EncoderDCMotor(self.rate, 360., **params)
        self.c = controller.motor_position_controller(Ts, Pos, self.m)
        self.c.target = 1.

    def loop(self, t):
        self.c.target = np.sin(t)
        self.m.u = self.c.control(self.m.get_position())
        self.m.step()
        return {
            'position': self.m.get_position(),
            'input': self.c.target,
            'error': self.c.error
        }

if __name__ == '__main__':
    stream = MotorControl(['position', 'input', 'error'], 20e-3)
    rtplot = signals.RealTimePlot(stream, interval=30, autoscale=True,
                                  xlim=300., keep_data=300, ts=0.01,
                                  ylim=(0, 150))
    rtplot.show()
    stream.start()
    while raw_input('Q to exit\n\r> ').lower() != 'q':
        pass
    stream.stop()
