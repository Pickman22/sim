import numpy as np
import copy
import logging
import matplotlib.pyplot as plt
from utils import step, create_axes

logging.basicConfig(level = logging.DEBUG)

def motor_sim(m, t0, tf, ts, *args, **kwargs):
    on_input = kwargs.get('on_input', None)
    u = kwargs.get('input', 0.)
    vin = [u]
    tlen = (tf - t0) / ts
    t = t0
    x = m.x
    while t < tf:
        if on_input is not None:
            u = on_input(x[:, -1], t, *args)
        m.step(u)
        x = np.hstack((x, m.x))
        t += ts
        vin.append(u)
    t = np.linspace(t0, tf, x.shape[1])
    return t, x.T, vin

class DCMotor(object):

    def __init__(self, ts, R = 1., L = 1., J = 1., Kt = 1., Kb = 1., Kf = 1., 
    x0 = [0., 0., 0.]):
        self.x = np.zeros([3, 1])
        self.A = np.array([
            [0.,          1.,       0.], 
            [0.,    -Kf / J ,  Kt / J ], 
            [0.,    -Kb / L , -R  / L ]
        ])
        self.B = np.array([
            [0.], 
            [0.], 
            [1. / L]
        ])
        self.x = np.array(x0).reshape(3, 1)
        self.u = 0.
        self.ts = ts
        logging.debug('A:\n\r%s', self.A)
        logging.debug('B:\n\r%s', self.B)
        self.dim = self.x.size
    
    def step(self, u = None):
        if u is not None:
            self.u = u
        dx = self.dynamics(self.u)
        self.x = step(dx, self.x, self.ts)
        return self.x
    
    def dynamics(self, u):
        try:
            self.A.reshape([3, 3])
            self.B.reshape([3, 1])
            self.x.reshape([3, 1])
            return self.A.dot(self.x) + u * self.B
        except:
            raise ValueError('System dynamics dimension mismatch')
            return np.zeros([3, 1])
        
if __name__ == '__main__':

    ti = 0.
    tf = 3.
    ts = 0.002
    
    params = {
        'kf': 0.1, # Friction Coefficient.
        'kb': 0.01, # Back-emf Coefficient.
        'L': 0.5, # Armature Inductance.
        'R': 0.1, # Resistance.
        'J': 0.01, # Rotor Inertia.
        'kt': 0.01, # Torque Constant.
    }
    
    m = DCMotor(params)
    t, x = motor_sim(m, ti, tf, ts, plot = False, input = 24.)
    plt.plot(t, x)
    plt.legend(['position', 'veloctiy', 'current'], loc = 'best')
    plt.show()
