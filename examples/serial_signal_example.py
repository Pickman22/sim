import argparse
from signals import signals

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Serial Port Monitor', )
    parser.add_argument('-p', '--port', action='store', dest='port',
                        default='/dev/ttyACM0')
    parser.add_argument('-b', '--baudrate', action='store', dest='baudrate',
                        default=115200, type=int)
    parser.add_argument('-n', '--names', action='append', dest='names',
                        default=[], help='Signal labels')
    args = parser.parse_args()
    stream = signals.SerialSignal(args.names, port=args.port,
                                  baudrate=args.baudrate)
    stream.start()
    r = signals.RealTimePlot(stream, xlim=300., keep_data=300, ts=0.01,
                             ylim=(-2, 2))
    # r.animation.save('basic_animation.mp4', fps=30,
    #                  extra_args=['-vcodec', 'libx264'])
    r.show()
    print '####################################################################'
    print '######################### El Robotista #############################'
    print '####################################################################'
    print '### Plot signals recieved through Serial Port in real time. V1.0 ###'
    print '####################################################################'
    while True:
        print 'Hit Q to exit.'
        if raw_input('> ').lower() == 'q':
            break
    print 'Bye.'
    stream.stop()
