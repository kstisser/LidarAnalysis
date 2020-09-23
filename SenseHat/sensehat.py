from sense_hat import SenseHat
from time import sleep

#Purpose of this class is to read temperature and humidity readings from a 
#raspberry pi sense hat

class SensorReadings:
    def __init__(self):
        self.sense = SenseHat()

    def getTemperature(self):
        return self.sense.get_temperature()

    def getHumidity(self):
        return self.sense.get_humidity()

    def printTempHumidityReadings(self):
        while True:
            print("Temperature: ")
            print(self.getTemperature())
            print("Humidity" )
            print(self.getHumidity())
