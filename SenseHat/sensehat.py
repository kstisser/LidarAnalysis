#!/usr/bin/env python
from sense_hat import SenseHat
from time import sleep
import numpy as np
import rospy
from std_msgs.msg import Float32

#Purpose of this class is to read temperature and humidity readings from a 
#raspberry pi sense hat

class SensorReadings:
    def __init__(self):
        self.sense = SenseHat()
        self.sense.clear()
        self.temperatures = np.array([])
        self.humidities = np.array([])

    def getTemperature(self):
        temp = self.sense.get_temperature()
        return temp

    def getHumidity(self):
        humidity = self.sense.get_humidity()
        return humidity

    def printTempHumidityReadings(self):
        while True:
            print("Temperature: ")
            print(self.getTemperature())
            print("Humidity" )
            print(self.getHumidity())

    def getRunningAverage(self):
        while True:
            temp = self.getTemperature()
            self.temperatures = np.append(self.temperatures, temp)
            print("Temperature moving average: ")
            print(self.temperatures.mean())
            humid = self.getHumidity()
            self.humidities = np.append(self.humidities, humid)
            print("Humidities moving average: ")
            print(self.humidities.mean())

if __name__ == '__main__':
    try:
        sr = SensorReadings()
        tempPub = rospy.Publisher('temperature', Float32, queue_size=10)
        humidPub = rospy.Publisher('humidity', Float32, queue_size=10)

        rospy.init_node('tempHumid', anonymous=True)
        rate = rospy.Rate(10) #10 Hz

        while not rospy.is_shutdown():
            tempPub.publish(sr.getTemperature())
            humidPub.publish(sr.getHumidity())
            rate.sleep() 
    except rospy.ROSInterruptException:
        pass
