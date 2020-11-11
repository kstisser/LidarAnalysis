#reference for code at arcanesciencelab.wordpress.com
#Working with the adafruit circuit playground express using the raspberry pi
import glob, serial, time, signal, sys


class CPExpressCommunicator:
    def __init__(self, deviceName, serial):
        self.device = glob.glob(deviceName, recursive=False)[0]
        self.serUSB = serial.Serial(self.device, serial, timeout=1)

    def sigint_handler(self, signum, frame):
        self.serUSB.close()
        print()
        sys.exit(0)

    def startConnection(self):
        signal.signal(signal.SIGINT, self.sigint_handler)
        self.serUSB.close()
        self.serUSB.open()

    def publishLight(self):
        print("Starting to get light readings")
        while True:
            usbdata = serUSB.read_until()
            if len(usbdata) > 0:
                print(usbdata.decode('UTF-8').rstrip())

if __name__ == '__main__':
    try:
        communicator = CPExpressCommunicator("/dev/ttyACM*", 115200)
        communicator.startConnection()
        communicator.publishLight()
    except rospy.ROSInterruptException:
        pass
