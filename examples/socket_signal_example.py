import signals

if __name__ == '__main__':
    stream = signals.SocketSignal(['data'], 'localhost', 1234)
    r = signals.RealTimePlot(stream, xlim=300., keep_data=300, ts=0.01,
                             ylim=(-2, 2))
    r.show()
    stream.start()
    while True:
        print 'Hit Q to exit.'
        if raw_input('> ').lower() == 'q':
            break
    print 'Bye.'
    stream.stop()
