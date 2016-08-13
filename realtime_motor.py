from signals import Buffer, RealTimePlot
from motor import DCMotor
import logging
import threading

logging.basicConfig(level = logging.DEBUG)
logging.getLogger().setLevel(logging.WARN)

class RealTimeMotor(DCMotor):
    
    def __init__(self, ts, stream, **kwargs):
        logging.debug('RealTimeMotor args: %s', kwargs)
        DCMotor.__init__(self, **kwargs)
        self.ts = ts
        self.buffer = stream
        self.is_running = False
        
    def _stream(self):
        x = self.step(self.ts, self.u)
        self.buffer.add_dict_data({'w': x[1], 'i': x[2]})
        logging.debug('%s', x)
        if self.is_running:
            threading.Timer(self.ts, self._stream).start()
        
    def run(self):
        if self.is_running:
            return
        self.is_running = True
        self._stream()
        
    def stop(self):
        self.is_running = False
        

if __name__ == '__main__':
    stream = Buffer(['i', 'w'])
    rtplot = RealTimePlot(stream, interval = 30, autoscale = False, xlim = 300., keep_data = 300, ts = 0.01, ylim = (0, 150))
    rtplot.show()
    rtmotor = RealTimeMotor(40 / 1000., stream, R = 1., Kf = 0.1, Kb = 0.01, J = 0.01, L = 0.5, Kt = 0.01)
    rtmotor.run()
    while True:
        ch = raw_input('Type Q to exit. Set the input volgate.\n\r> ')
        if ch.lower() == 'q':
            break
        else:
            try:
                u = float(ch)
                rtmotor.u = u
                print 'New input voltage: {}'.format(rtmotor.u)
            except ValueError:
                print 'Not a valid input voltage.'
    rtmotor.stop()
    print 'Bye.'
