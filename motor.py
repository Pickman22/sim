import numpy as np
import copy
import logging
import matplotlib.pyplot as plt

logging.basicConfig(level = logging.DEBUG)

def motor_sim(m, t0, tf, ts, on_input = None, plot = True):
    tlen = (tf - t0) /ts
    t = t0
    x = m.x
    u = 100.
    while t < tf:
        if on_input is not None:
            u = on_input(x[:, -1], t)
        m.step(ts, u)
        x = np.hstack((x, m.x))
        t += ts
    t = np.linspace(t0, tf, x.shape[1])
    if plot:
        plt.plot(t, x[1,:].T)
        #plt.legend(['Position', 'Velocity', 'Current'], loc = 'best')
        plt.show()
    return t, x.T

def step(dx, x, ts):
    if ts <=0.:
        raise ValueError('Timestep must be positive')
        return np.zeros(x.shape)
    try:
        return dx * ts + x
    except:
        raise ValueError('System dimension mismatch')
        return np.zeros(x.shape)

class DCMotor(object):

    def __init__(self, ts = 0.02, Kf = 1., Kb = 1., R = 1., L = 1., Kt = 1., 
    J = 1., x0 = [0., 0., 0.]):
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
    
    def step(self, ts = None, u = None):
        if u is not None:
            self.u = u
        if ts is not None:
            self.ts = ts
        dx = self.dynamics(self.u)
        self.x = step(dx, self.x, ts)
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
    
    vin = 1.
    
    params = {
        'kf': 0.1, # Friction Coefficient.
        'kb': 0.01, # Back-emf Coefficient.
        'L': 0.5, # Armature Inductance.
        'R': 0.1, # Resistance.
        'J': 0.01, # Rotor Inertia.
        'kt': 0.01, # Torque Constant.
    }
    
    x0 = {
        'position': 0.,
        'velocity': 0.,
        'current': 0.
    }

    m = DCMotor(params = params)
    t, x = motor_sim(m, ti, tf, ts)
    plt.plot(t, x)
    plt.legend(['position', 'veloctiy', 'current'], loc = 'best')
    plt.show()
