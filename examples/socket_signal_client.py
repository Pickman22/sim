import socket
import signal
import numpy.random as random
import threading
import json

quit = False


def send_data(sock):
    global quit
    data = {'data': random.random()}
    sock.sendall(json.dumps(data))
    if not quit:
        threading.Timer(20e-3, send_data, (sock,)).start()


if __name__ == '__main__':
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('localhost', 1234))
    send_data(sock)
    while raw_input('Q to exit\n\r> ').lower() != 'q':
        pass
    sock.close()
    print('Bye.')
