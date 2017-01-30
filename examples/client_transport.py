import transport
import binascii
import struct
import controller
import logging
import numpy as np
import copy

logging.basicConfig(level=logging.DEBUG)
logging.getLogger().setLevel(logging.DEBUG)

class Remote_PIDController(controller.Motor_PID_Controller):

    GET_ERROR_CMD = 0x13
    GET_OUTPUT_CMD = 0x10
    GET_FEEDBACK_CMD = 0x21
    GET_TARGET_CMD = 0x70

    ''' The step command is implemented as follows:
    Due to the fact that the system may or not have sensors, observers attached,
    an identifier is attached before the data for every variable. This way,
    the communication end can detect the data that is being provided. A step
    command has the following format:

        | STEP_CMD | X1_ID | X1_32 | X2_ID | X2_32 | X3_ID | X3_32 | ...
        | SENS_P_ID | SENS_P_32 | SENS_W_ID | SENS_W_32 | SENS_I_ID | SENS_I_32 | ...
        | OBS_P_ID | OBS_P_32 | OBS_W_ID | OBS_W_32 | OBS_I_ID | OBS_I_32 | ...
        | TIME_ID | TIME_32 |

    If the ID fied for a signal is not present. It means that the
    state / sensor / obsever is not available. Each value is represented as a
    32-bit integer. A scaling of 1e3 is assumed for floating point signals.
    '''

    STEP_CMD = 0x80

    X1_ID = 0x00
    X2_ID = 0x01
    X3_ID = 0x02
    X_ID = [X1_ID, X2_ID, X3_ID]

    SENS_P_ID = 0x10
    SENS_W_ID = 0x11
    SENS_I_ID = 0x12
    SENS_ID = [SENS_P_ID, SENS_W_ID, SENS_I_ID]

    OBS_P_ID = 0x20
    OBS_W_ID = 0x21
    OBS_I_ID = 0x22
    OBS_ID = [OBS_P_ID, OBS_W_ID, OBS_I_ID]

    TIME_ID = 0x30
    TARGET_ID = 0x31

    def __init__(self, transport, **kwargs):
        self.transport = transport

        self._signal_dict = {self.X1_ID: np.nan,
                             self.X2_ID: np.nan,
                             self.X3_ID: np.nan,
                             self.SENS_P_ID: np.nan,
                             self.SENS_W_ID: np.nan,
                             self.SENS_I_ID: np.nan,
                             self.OBS_P_ID: np.nan,
                             self.OBS_W_ID: np.nan,
                             self.OBS_I_ID: np.nan,
                             self.TARGET_ID: np.nan,}

        self._cmd_dict = {self.STEP_CMD: self.step,
                          self.GET_ERROR_CMD: self.get_error,
                          self.GET_OUTPUT_CMD: self.get_output,
                          self.GET_FEEDBACK_CMD: self.get_feedback,
                          self.GET_TARGET_CMD: self.get_target,}

        super(controller.Motor_PID_Controller, self).__init__(**kwargs)

    def start(self):
        if self.transport.is_connected:
            msg = self.transport.read()
            cmd, = struct.unpack('>B', msg[0])
            if self._cmd_dict.has_key(cmd):
                logging.debug('Executing: {}'.format(self._cmd_dict[cmd]))
                if cmd == self.STEP_CMD:
                    assert(len(msg) > 1)
                    self.step(msg[1:])
                else:
                    assert(len(msg) == 1)
                    self._cmd_dict[cmd]()
            else:
                logging.warning('Unknown command {}'.format(cmd))

    def step(self, bytes):
        # Each signal is made of its ID + 4 bytes of data. If the length
        # of the byte array data is not divisible by 5, it means that we are
        # getting corrupt data.
        assert((len(bytes) % 5) == 0)
        #TODO: Convert bytes into actual dataaaaaaaaaaaa.

if __name__ == '__main__':
    message = struct.pack('>i', 24000)
    tcp = transport.TCPClientTransport()

    remote_controller = Remote_PIDController(tcp)

    while tcp.is_connected:
        #print binascii.hexlify(tcp.read())
        #msg = tcp.read()
        remote_controller.start()
        try:
            remote_controller.transport.write(message)
        #try:
        #    tcp.write(message)
        except:
            pass
    remote_controller.transport.disconnect()
    #tcp.disconnect()
