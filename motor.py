import numpy as np
import copy
import logging
import matplotlib.pyplot as plt

logging.basicConfig(level = logging.DEBUG)

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

    __params_default = {
        'R' :  1., # Resistance.
        'L' :  1., # Inductance.
        'Kt':  1., # Torque Constant.
        'J' :  1., # Inertia.
        'Kb':  1., # Back-emf Constant.
        'Kf':  1.,  # Friction Coefficient.
        'ts':  0.02 # Sampling Time.
    }
    
    __x0_default = {
        'position' :0.,
        'velocity' :0.,
        'current'  :0.
    }

    def __init__(self, **kwargs):
        self.params = copy.deepcopy(self.__params_default)
        try:
            self.params.update(kwargs['params'])
            for k, v in self.params:
                if v < 0.:
                    self.params['k'] = 0.
                    raise ValueError('Motor para meter %s must be positive.', k)
        except:
		    logging.info('Motor parameters not provided. Using default.')
		
        x0 = copy.deepcopy(self.__x0_default)
        try:
            x0.update(kwargs['initial_conditions'])
        except:
            logging.info('Motor initial conditions not provided. Setting as zero.')
        
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
        
        self.dim = self.x.size
    
    def step(self, u = 0.):
        dx = self.dynamics(u)
        self.x = step(dx, self.x, self.params['ts'])
        return self.x
    
    def dynamics(self, u):
        dx = np.zeros([3, 1])
        try:
            self.A.reshape([3, 3])
            self.B.reshape([3, 1])
            self.x.reshape([3, 1])
            dx = self.A.dot(self.x) + u * self.B
        except:
            raise ValueError('System dimension mismatch')
            dx = np.zeros([3, 1])
        return dx
        

        
if __name__ == '__main__':

    def sim(sys, t0, tf, ts):
        tlen = (tf - t0) /ts
        t = t0
        x = sys.x
        while t < tf:
            sys.step(100)
            x = np.hstack((x, sys.x))
            t += ts
        t = np.linspace(t0, tf, x.shape[1])
        return t, x.T
    
    params = {
        'J': 0.1,
        'Kb': 0.1,
        'Kt': 0.1,
        'R': 1.,
        'L': 0.5,
        'Kf': 0.01
    }
    
    x0 = {
        'position': 0.,
        'velocity': 0.,
        'current': 0.
    }

    m = DCMotor(params = params, initial_conditions = x0)
    t, x = sim(m, 0, 10., 0.02)
    
    plt.plot(t, x)
    plt.legend(['Position', 'Velocity', 'Current'], loc = 'best')
    plt.title('Motor Response')
    plt.xlabel('Time {s}')
    plt.ylabel('States')
    plt.show()
