#!/usr/bin/env python
from time import sleep
import numpy as np
import rospy
from std_msgs.msg import Float32
import cv2
import matplotlib.pyplot as plt
import json
import atexit

class DataAccumulator:
    def __init__(self, filepath):
        self.temperature = []
        self.humidity = []
        self.light = []
        with open(filepath) as f:
            self.jsonData = json.load(f)
        self.filepath = filepath
        self.jsonData["temperature"] = []
        self.jsonData["humidity"] = []
        self.jsonData["light"] = []

    def callbackTemp(self, msg):
        print(msg)
        self.temperature.append(float(msg.data))

    def callbackHumidity(self, msg):
        print(msg)
        self.humidity.append(float(msg.data))

    def callbackLight(self, msg):
        print(msg)
        self.light.append(float(msg.data))
    
    def saveData(self):
        self.jsonData["temperature"] = (self.temperature)
        self.jsonData["humidity"] = (self.humidity)
        self.jsonData["light"] = (self.light)
        '''for t in self.temperature:
            self.jsonData["temperature"].append(self.temperature[t])

        for h in self.humidity:
            self.jsonData["humidity"].append(self.humidity[h])

        for l in self.light:
            self.jsonData["light"].append(self.light[l])'''

        with open(self.filepath, "w") as outfile:
            json.dump(self.jsonData, outfile)
            #outfile.write(self.jsonData)


if __name__ == '__main__':
    try:
        sr = DataAccumulator("lidarPoints.json")
        atexit.register(sr.saveData)
        rospy.init_node('data_processor', anonymous=True)
        rospy.Subscriber("/temperature", Float32, sr.callbackTemp)
        rospy.Subscriber("/humidity", Float32, sr.callbackHumidity)
        rospy.Subscriber("/light", Float32, sr.callbackLight)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
