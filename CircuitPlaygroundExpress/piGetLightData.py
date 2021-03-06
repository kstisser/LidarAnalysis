#reference for code at arcanesciencelab.wordpress.com
#Working with the adafruit circuit playground express using the raspberry pi
import glob, serial, time, signal, sys
import rospy
from std_msgs.msg import Float32

class CPExpressCommunicator:
    def __init__(self, deviceName, serialNum):
        self.device = glob.glob(deviceName)[0]
        self.serUSB = serial.Serial(self.device, serialNum, timeout=1)

    def sigint_handler(self, signum, frame):
        self.serUSB.close()
        print()
        sys.exit(0)

    def startConnection(self):
        signal.signal(signal.SIGINT, self.sigint_handler)
        self.serUSB.close()
        self.serUSB.open()

    def getLightReading(self):
        #print("Starting to get light readings")
        while True:
            usbdata = self.serUSB.read_until()
            if len(usbdata) > 0:
                lightRtn = usbdata.decode('UTF-8').rstrip()
                print(lightRtn)
                return lightRtn

if __name__ == '__main__':
    try:
        communicator = CPExpressCommunicator("/dev/ttyACM*", 115200)
        communicator.startConnection()

        lightPub = rospy.Publisher('light', Float32, queue_size=10)
        rospy.init_node('lightNode', anonymous=True)
        rate = rospy.Rate(10) #10 Hz

        while not rospy.is_shutdown():
            light = communicator.getLightReading()
            lightPub.publish(light)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
