import copy

class MsgParser(object):
    START = 0x0A
    END = 0xA0

    _WAITING_START = 1
    _GETTING_PAYLOAD_SIZE = 2
    _GETTING_PAYLOAD = 3
    _EXPECTING_END = 4
    def __init__(self, has_message_callback = None):
        self.reset()
        self.has_message_callback = has_message_callback

    def parse(self, data):
        assert(type(data) is int)
        assert(0 <= data <= 255)

        if self._state == self._WAITING_START:
            #print('Waiting start.')
            assert(len(self._payload) == 0)
            assert(self._sz == 0)
            if data == self.START:
                self._state = self._GETTING_PAYLOAD_SIZE
            self.has_message = False

        elif self._state == self._GETTING_PAYLOAD_SIZE:
            #print('Getting payload size.')
            assert(len(self._payload) < 2)
            self._payload.append(data)
            if len(self._payload) == 2:
                self._sz = ((0xff & self._payload[0]) << 8) | self._payload[1]
                self._payload = []
                self._state = self._GETTING_PAYLOAD
            self.has_message = False

        elif self._state == self._GETTING_PAYLOAD:
            #print('Getting payload.')
            assert(self._sz > 0)
            self._payload.append(data)
            self._sz -= 1
            if self._sz == 0:
                self._state = self._EXPECTING_END
            self.has_message = False

        elif self._state == self._EXPECTING_END:
            #print('Expecting end.')
            assert(self._sz == 0)
            assert(len(self._payload) > 0)
            if data == self.END:
                #print('Got end.')
                if self.has_message_callback:
                    msg = copy.copy(self._payload)
                    self._payload = []
                    self.has_message_callback(msg)
                    self.has_message = False # Already delivered message.
                else:
                    #print('Message complete.')
                    self.has_message = True
            else:
                self.reset()
                raise RuntimeError('Invalod message. Expected frame END.')

        else:
            raise RuntimeError('Unknown state.')

    def reset(self):
        self._payload = []
        self._sz = 0
        self._state = self._WAITING_START
        self.has_message = False

    def get_message(self):
        if self.has_message:
            ret = copy.copy(self._payload)
        else:
            ret = []
        self.has_message = False
        return ret

    @classmethod
    def pack_data(cls, data):
        assert(len(data) > 0)
        assert(len(data) <= 0xffff)
        byte_data = bytearray(data)
        sz_msb = (len(byte_data) >> 8) & 0x00FF
        sz_lsb = len(byte_data) & 0x00fF
        assert(0 < sz_lsb <= 255) # At least one byte of real data.
        assert(0 <= sz_msb <= 255) # Most significant byte IS allowed to be zero.
        byte_data.insert(0, sz_lsb)
        byte_data.insert(0, sz_msb)
        byte_data.insert(0, cls.START)
        byte_data.append(cls.END)
        return byte_data
