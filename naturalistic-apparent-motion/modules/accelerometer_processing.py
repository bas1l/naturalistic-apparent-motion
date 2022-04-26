class AccelerometerProcessing:

    def __init__(self, _port):
        self.port = _port
        

    def ping(self, printMessages):
        print('Teensy: {}' .format(printMessages))