#!/usr/bin/python
import json
import os
import glob
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import colors
from matplotlib.colors import Normalize
from matplotlib.ticker import PercentFormatter
import models

def simulate1():        
    area = 0.18
    temperature = 21
    humidity = 35

    fig, ax = plt.subplots()
    fig.suptitle("Simulated Distance Variation")
    ax.set_xlabel("Distance")
    ax.set_ylabel("Lidar Points")
    error = 0
    numPoints = 0
    realPoints = []
    distances = []
    accumulatedPoints = 0
    for x in range(35):
        i = 0.25 + (0.25 * x)
        distance = i
        print(i)
        realNumberOfPoints = abs(models.getResultFromTunedEquation(area, temperature, humidity, distance))
        print(realNumberOfPoints)
        perfectPoints = models.getP(area, distance)
        print("Perfect:" + str(perfectPoints))
        realPoints.append(perfectPoints)
        distances.append(i)
        error = error + (perfectPoints - realNumberOfPoints)
        numPoints = numPoints + 1
        accumulatedPoints = accumulatedPoints + perfectPoints
        plt.plot( i,realNumberOfPoints, 'ro')

    plt.plot(distances, realPoints,color='green')
    aveError = error/numPoints
    aveAccumulated = accumulatedPoints/numPoints
    overall= aveError/aveAccumulated
    print("Ave error: " + str(aveError))
    print("Ave points: " + str(aveAccumulated))
    print("Overall error percent: " + str(overall))
    plt.show()
    
def simulate2():        
    area = 0.4
    humidity = 35

    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    fig.suptitle("Simulated Distance and Temperature Variation")
    ax.set_xlabel("Distance (m)")
    ax.set_ylabel("Number of Lidar Points")
    ax.set_zlabel("Temperature (C)")
    error = 0
    numPoints = 0
    realPoints = []
    calcPoints = []
    distances = []
    temperatures = []
    accumulatedPoints = 0
    for temp in range(30):
        for x in range(35):
            i = 0.25 + (0.25 * x)
            distance = i
            print(i)
            print("Temp: " + str(temp))
            realNumberOfPoints = (abs(models.getResultFromTunedEquation(area, temp, humidity, distance))/1000.0)
            print(realNumberOfPoints)
            perfectPoints = (models.getP(area, distance))/1000.0
            print("Perfect:" + str(perfectPoints))
            realPoints.append(perfectPoints)
            distances.append(i)
            calcPoints.append(realNumberOfPoints)
            temperatures.append(temp)
            error = error + (perfectPoints - realNumberOfPoints)
            numPoints = numPoints + 1
            accumulatedPoints = accumulatedPoints + perfectPoints
            plt.plot( [i],[realNumberOfPoints], [temp], 'ro')
            plt.plot([i],[perfectPoints],[temp],'gx')

    #print("Results:")
    #print(distances)
    #print(realPoints)
    #print(temperatures)
    #plt.plot(distances, realPoints, temperatures,color='green')
    print("Minimum points:")
    print(min(calcPoints))
    aveError = error/numPoints
    aveAccumulated = accumulatedPoints/numPoints
    overall= aveError/aveAccumulated
    print("Ave error: " + str(aveError))
    print("Ave points: " + str(aveAccumulated))
    print("Overall error percent: " + str(overall))
    plt.show()    
