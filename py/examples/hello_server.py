import transport

if __name__ == '__main__':
    ''' Simple code used to tests IPC to C code. Notice that the TCPServerTransport
    is just used to create a connection. After that the raw socket is used to
    send and receive data. Of course this is not intented to be used like this
    in the application. '''

    msg = 'Hello from python!'
    server = transport.TCPServerTransport(ip = 'localhost', port = 9090)
    print('Received from client: {}'.format(server.read()))
    print('Send to client: {}'.format(msg))
    server.write(msg)
    server.conn.close()
