import abc
import socket
import logging
import struct
import copy
import binascii

logging.basicConfig(level=logging.DEBUG)
logging.getLogger().setLevel(logging.DEBUG)

class Transport(object):

    START = struct.pack('>B', 0x0A)
    END = struct.pack('>B', 0xA0)
    SHUTDOWN = struct.pack('B', 0xAF)
    SHUTDOWN_MSG = START + struct.pack('>HB', 1, 0) + SHUTDOWN

    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def connect(self):
        pass

    @abc.abstractmethod
    def disconnect(self):
        pass

    @abc.abstractmethod
    def _write_bytes(self):
        pass

    def write(self, bytes):
        ''' @data is assumed to be a string of data or bytearray containing the
        data. Is responsibility of the application to give the data to the
        transport layer. This class will stuff the data with 16-bit frame
        denoting the length (big-endian format), a START byte and an END of data
        byte. '''

        if not self.is_connected:
            raise RuntimeError('Cannot write. Connect first!')
        packed_data = self.pack_data(bytes)
        self._write_bytes(packed_data)

    @abc.abstractmethod
    def _read_bytes(self):
        pass

    def read(self):
        if not self.is_connected:
            raise RuntimeError('Cannot read. Connect first!')

        # Read the first 3 bytes. The first byte contains the START byte.
        # The remaining bytes are the length of the data that will be received
        # in big endian format: MSB first.
        data = self._read_bytes(3)
        assert(data[0] == self.START)

        # Convert the data-bytes to the number of bytes. '>H' converts to
        # unsigned short (16-bit) in big endian format.
        datalen, = struct.unpack('>H', data[1:])

        # Now that we know how much data to expect, prepare to read it.
        data = self._read_bytes(datalen)

        # The next byte MUST be the END byte. Otherwise we error out.
        end = self._read_bytes(1)
        #assert(end == self.END or end == self.SHUTDOWN)
        logging.debug('Received data: ' + binascii.hexlify(data) + ', end: ' + binascii.hexlify(end))
        if end == self.SHUTDOWN:
            self.disconnect()

        # The data is returned as is. It is responsibility of the application
        # to give meaning to the received data.
        return copy.copy(data)

    @abc.abstractproperty
    def is_connected(self):
        pass

    @classmethod
    def pack_data(cls, data):
        assert(type(data) is str)
        assert(0 < len(data) <= 0xFFFF)
        datalen = struct.pack('>H', len(data))
        return  cls.START + datalen +  data + cls.END

class TCPTransport(Transport):

    def __init__(self, ip = 'localhost', port = 5050):
        self.is_connected = False
        self.connect(ip, port)

    @abc.abstractmethod
    def connect(self):
        pass

    def is_connected(self):
        return self.is_connected

    def disconnect(self):
        ''' Let the receiver we are shutting down the comm channel. This is a
        special message with format:
            SHUTDOWN_CMD = [START | 0x00 | 0x01| 0x00 | SHUTDOWN]
        '''
        if self.is_connected:
            # If disconnect is called while still connected it means that we are
            # the ones shutting down the channel. Let the other side we are
            # stopping.
            self._write_bytes(self.SHUTDOWN_MSG)
            self.is_connected = False
            self.conn.shutdown(socket.SHUT_RDWR)
        self.conn.close()
        logging.debug('Connection closed.')

    def _write_bytes(self, data):
        if not self.is_connected:
            raise RuntimeError('Cannot write. Connect first!')
        logging.debug('Sending: ' + binascii.hexlify(data))
        self.conn.sendall(data)

    def _read_bytes(self, nbytes):
        if not self.is_connected:
            raise RuntimeError('Cannot read. Connect first!')
        return self.conn.recv(nbytes)

class TCPServerTransport(TCPTransport):

    def connect(self, ip = 'localhost', port = 5050):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        logging.debug('Attempting connection to {}:{}'.format(ip, port))
        sock.bind((ip, port))
        logging.debug('Socket bound. Listening for client...')
        sock.listen(1)
        self.conn, self.addr = sock.accept()
        logging.debug('Accepted connection from {}'.format(self.addr))
        self.is_connected = True

class TCPClientTransport(TCPTransport):

    def connect(self, ip = 'localhost', port = 5050):
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        logging.debug('Attempting connection to {}:{}'.format(ip, port))
        self.conn.connect((ip, port))
        logging.debug('Connected to {}'.format(ip))
        self.is_connected = True
