from sense_hat import SenseHat
from time import sleep
import numpy as np

#Purpose of this class is to read temperature and humidity readings from a 
#raspberry pi sense hat

class SensorReadings:
    def __init__(self):
        self.sense = SenseHat()
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
