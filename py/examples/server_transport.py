from transport import TCPServerTransport

if __name__ == '__main__':
    server = TCPServerTransport()
    print('Message Received: {}'.format(server.read()))
    server.disconnect()
