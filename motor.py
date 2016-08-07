import numpy as np
import copy
import logging
import matplotlib.pyplot as plt

logging.basicConfig(level = logging.DEBUG)

def motor_sim(m, t0, tf, ts, on_input = None):
    tlen = (tf - t0) /ts
    t = t0
    x = m.x
    u = 100.
    while t < tf:
        if on_input is not None:
            u = on_input(x, t)
        m.step(ts, u)
        x = np.hstack((x, m.x))
        t += ts
    t = np.linspace(t0, tf, x.shape[1])
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

    __default_params = {
        'R' :  1., # Resistance.
        'L' :  1., # Inductance.
        'Kt':  1., # Torque Constant.
        'J' :  1., # Inertia.
        'Kb':  1., # Back-emf Constant.
        'Kf':  1.,  # Friction Coefficient.
    }
    
    __default_x0 = {
        'position' :0.,
        'velocity' :0.,
        'current'  :0.
    }


    def __init__(self, **kwargs):
        self.params = copy.deepcopy(self.__default_params)
        x0 = copy.deepcopy(self.__default_x0)
        try:
            self.params.update(kwargs['params'])
            logging.debug('Motor params: %s', self.params)
        except Exception as e:
            logging.info('%s. Using default parameters', e.message)

        for k, v in self.params.iteritems():
            if v < 0.:
                raise ValueError('Motor parameter %s must be positive. Setting to one.', k);
                self.params[k] = 1.
        
        self.x = np.zeros([3, 1])
        
        self.A = np.array([
            [0.,    1.,                                         0.                                  ], 
            [0.,    -self.params['Kb'] / self.params['J'],      self.params['Kt'] / self.params['J']], 
            [0.,    -self.params['Kt'] / self.params['L'],      -self.params['R'] / self.params['L']]
        ])
        
        self.B = np.array([
            [0.], 
            [0.], 
            [1. / self.params['L']]
        ])
        self.x[0] = x0['position']
        self.x[1] = x0['velocity']
        self.x[2] = x0['current']
        logging.debug('A:\n\r%s', self.A)
        logging.debug('B:\n\r%s', self.B)
        self.dim = self.x.size
    
    def step(self, ts, u = 0.):
        dx = self.dynamics(u)
        self.x = step(dx, self.x, ts)
        return self.x
    
    def dynamics(self, u):
        try:
            self.A.reshape([3, 3])
            self.B.reshape([3, 1])
            self.x.reshape([3, 1])
            return self.A.dot(self.x) + u * self.B
        except:
            raise ValueError('System dimension mismatch')
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
