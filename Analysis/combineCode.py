#!/usr/bin/python
import json
import os
import glob
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import scipy.interpolate as interpolate
import plotter
import reconstructImage
import models

def traverseDirectoriesAndProcess(filepath):
    masterJSON = {}
    masterJSON['area'] = []
    masterJSON['distance'] = []
    masterJSON['objectTemperature'] = []
    masterJSON['red'] = []
    masterJSON['green'] = []
    masterJSON['blue'] = []
    masterJSON['temperature'] = []
    masterJSON['humidity'] = []
    masterJSON['light'] = []
    masterJSON['objectTemperature'] = []
    masterJSON['numLidarPoints'] = []
    
    testJSON = {}
    testJSON['area'] = []
    testJSON['distance'] = []
    testJSON['objectTemperature'] = []
    testJSON['red'] = []
    testJSON['green'] = []
    testJSON['blue'] = []
    testJSON['temperature'] = []
    testJSON['humidity'] = []
    testJSON['light'] = []
    testJSON['objectTemperature'] = []
    testJSON['numLidarPoints'] = []
    
    trainingJSON = {}
    trainingJSON['area'] = []
    trainingJSON['distance'] = []
    trainingJSON['objectTemperature'] = []
    trainingJSON['red'] = []
    trainingJSON['green'] = []
    trainingJSON['blue'] = []
    trainingJSON['temperature'] = []
    trainingJSON['humidity'] = []
    trainingJSON['light'] = []
    trainingJSON['objectTemperature'] = []
    trainingJSON['numLidarPoints'] = []    
    
    cubeJSON = {}
    cubeJSON['area'] = []
    cubeJSON['distance'] = []
    cubeJSON['objectTemperature'] = []
    cubeJSON['red'] = []
    cubeJSON['green'] = []
    cubeJSON['blue'] = []
    cubeJSON['temperature'] = []
    cubeJSON['humidity'] = []
    cubeJSON['light'] = []
    cubeJSON['objectTemperature'] = []
    cubeJSON['numLidarPoints'] = []
    
    handwarmerJSON = {}
    handwarmerJSON['area'] = []
    handwarmerJSON['distance'] = []
    handwarmerJSON['objectTemperature'] = []
    handwarmerJSON['red'] = []
    handwarmerJSON['green'] = []
    handwarmerJSON['blue'] = []
    handwarmerJSON['temperature'] = []
    handwarmerJSON['humidity'] = []
    handwarmerJSON['light'] = []
    handwarmerJSON['objectTemperature'] = []
    handwarmerJSON['numLidarPoints'] = []    
    filepath = 'C:/Users/fm873/Documents/Repos/LidarAnalysis/Results'
    print(filepath)
    
    for root, subdirs, files in os.walk(filepath):
        #print files
        for f in files:
            if f == 'lidarPoints.json':
                #Read in json
                with open(os.path.join(root,f)) as fd:
                    jsonData = json.load(fd)

                print(root)
                #print(np.mean(jsonData['pointNumberByFrame']))
                #reconstructImage.reconstructImage(root,jsonData)
                #make all fields in the file the same length so they can be added to a data frame
                pointNumberSize = len(jsonData['pointNumberByFrame'])
                
                tempSize = len(jsonData['temperature'])
                humidSize = len(jsonData['humidity'])
                lightSize = len(jsonData['light'])
                
                '''print("num points")
                print(pointNumberSize)
                print("temp size")
                print(tempSize)
                print("humid size")
                print(humidSize)
                print("light size")
                print(lightSize)'''

                minVal = min(tempSize, humidSize)
                diff = abs(tempSize - humidSize)
                #if minVal != 0:
                #    maxLength = minVal
                #else:
                maxLength = min([tempSize,humidSize,lightSize,pointNumberSize])
                
                newTemp = {}
                newHumid = {}
                newLight = {}
                newPoints = {}
                
                #interpolate data based on number of points
                #f = interpolate.interp1d(jsonData['temperature'], jsonData['pointNumberByFrame'])
                minTemp = min(jsonData['temperature'])
                maxTemp = max(jsonData['temperature'])
                #inttemp = np.arrange
                #intFrameNum
                
                if len(jsonData['temperature']) > maxLength:
                    newTemp = (jsonData['temperature'])[:maxLength]
                else:
                    newTemp = jsonData['temperature']
                    while(len(newTemp) < maxLength):
                        newTemp.append(0)

                if len(jsonData['humidity']) > maxLength:
                    newHumid = (jsonData['humidity'])[:maxLength]
                else:
                    newHumid = jsonData['humidity']
                    while(len(newHumid) < maxLength):
                        newHumid.append(0)

                if len(jsonData['light']) > maxLength:
                    newLight = (jsonData['light'])[:maxLength]
                else:
                    newLight = (jsonData['light'])
                    while(len(newLight) < maxLength):
                        newLight.append(0)
                        
                if len(jsonData['pointNumberByFrame']) > maxLength:
                    newPoints = (jsonData['pointNumberByFrame'])[:maxLength]
                else:
                    newPoints = (jsonData['pointNumberByFrame'])
                    while(len(newPoints) < maxLength):
                        newPoints.append(0)        

                '''print("Max:")
                print(maxLength)
                print("temp size:")
                print(len(newTemp))
                print("humid size:")
                print(len(newHumid))
                print("light size:")
                print(len(newLight))
                print("point num size:")
                print(len(newPoints))'''

                width = jsonData["objectSize"]["width"]
                height = jsonData["objectSize"]["height"]
                area = width * height
                areaList = [area] * maxLength

                distance = jsonData["actualDistance"]
                distanceList = [distance] * maxLength

                objectTemp = jsonData["objectTemperature"]
                objectTempList = [objectTemp] * maxLength

                redPixel = jsonData["objectColor"]["r"]
                bluePixel = jsonData["objectColor"]["b"]
                greenPixel = jsonData["objectColor"]["g"]
                redList = [redPixel] * maxLength
                blueList = [bluePixel] * maxLength
                greenList = [greenPixel] * maxLength
                
                pointNum = jsonData["pointNumberByFrame"]

                newJsonData = {}
                newJsonData["area"] = areaList
                newJsonData["distance"] = distanceList
                newJsonData["objectTemperature"] = objectTempList
                newJsonData["red"] = redList
                newJsonData["blue"] = blueList
                newJsonData["green"] = greenList
                newJsonData["temperature"] = newTemp
                newJsonData["humidity"] = newHumid
                newJsonData["light"] = newLight
                newJsonData["numLidarPoints"] = newPoints
                
                #plo = plotter.Plotter(newJsonData)
                #plo.plot5D(root)
                percentageJSON = getPercentageOfExpectedPoints(newJsonData)
                try:
                    '''plo = plotter.Plotter(percentageJSON)
                    plo.plot3dAreaDistance(root)
                    plo.plotAndSaveAllData(root, "Scatter_Percentages", "numLidarPoints")
                    plo.plot1DPoints()
                    plo.plotHistograms(root)'''
                except:
                    print("Skipping plotting this one")

                for i in range(maxLength):
                    masterJSON["area"].append(newJsonData["area"][i])
                    masterJSON["distance"].append(newJsonData["distance"][i])
                    masterJSON["objectTemperature"].append(newJsonData["objectTemperature"][i])
                    masterJSON["red"].append(newJsonData["red"][i])
                    masterJSON["green"].append(newJsonData["green"][i])
                    masterJSON["blue"].append(newJsonData["blue"][i])
                    masterJSON["temperature"].append(newJsonData["temperature"][i])
                    masterJSON["humidity"].append(newJsonData["humidity"][i])
                    masterJSON["light"].append(newJsonData["light"][i])
                    masterJSON["numLidarPoints"].append(newJsonData["numLidarPoints"][i])

                if "Cubes" in root:
                    for i in range(maxLength):
                        cubeJSON["area"].append(newJsonData["area"][i])
                        cubeJSON["distance"].append(newJsonData["distance"][i])
                        cubeJSON["objectTemperature"].append(newJsonData["objectTemperature"][i])
                        cubeJSON["red"].append(newJsonData["red"][i])
                        cubeJSON["green"].append(newJsonData["green"][i])
                        cubeJSON["blue"].append(newJsonData["blue"][i])
                        cubeJSON["temperature"].append(newJsonData["temperature"][i])
                        cubeJSON["humidity"].append(newJsonData["humidity"][i])
                        cubeJSON["light"].append(newJsonData["light"][i])
                        cubeJSON["numLidarPoints"].append(newJsonData["numLidarPoints"][i])
                elif "Handwarmers" in root:
                    for i in range(maxLength):
                        handwarmerJSON["area"].append(newJsonData["area"][i])
                        handwarmerJSON["distance"].append(newJsonData["distance"][i])
                        handwarmerJSON["objectTemperature"].append(newJsonData["objectTemperature"][i])
                        handwarmerJSON["red"].append(newJsonData["red"][i])
                        handwarmerJSON["green"].append(newJsonData["green"][i])
                        handwarmerJSON["blue"].append(newJsonData["blue"][i])
                        handwarmerJSON["temperature"].append(newJsonData["temperature"][i])
                        handwarmerJSON["humidity"].append(newJsonData["humidity"][i])
                        handwarmerJSON["light"].append(newJsonData["light"][i])
                        handwarmerJSON["numLidarPoints"].append(newJsonData["numLidarPoints"][i])
                else:
                    print("Didn't recognize Cubes or Handwarmers in path!")

                if len(newJsonData["green"]) > 0 and newJsonData["green"][0] == 212:
                    for i in range(maxLength):
                        testJSON["area"].append(newJsonData["area"][i])
                        testJSON["distance"].append(newJsonData["distance"][i])
                        testJSON["objectTemperature"].append(newJsonData["objectTemperature"][i])
                        testJSON["red"].append(newJsonData["red"][i])
                        testJSON["green"].append(newJsonData["green"][i])
                        testJSON["blue"].append(newJsonData["blue"][i])
                        testJSON["temperature"].append(newJsonData["temperature"][i])
                        testJSON["humidity"].append(newJsonData["humidity"][i])
                        testJSON["light"].append(newJsonData["light"][i])
                        testJSON["numLidarPoints"].append(newJsonData["numLidarPoints"][i])                    
                else:
                    for i in range(maxLength):
                        trainingJSON["area"].append(newJsonData["area"][i])
                        trainingJSON["distance"].append(newJsonData["distance"][i])
                        trainingJSON["objectTemperature"].append(newJsonData["objectTemperature"][i])
                        trainingJSON["red"].append(newJsonData["red"][i])
                        trainingJSON["green"].append(newJsonData["green"][i])
                        trainingJSON["blue"].append(newJsonData["blue"][i])
                        trainingJSON["temperature"].append(newJsonData["temperature"][i])
                        trainingJSON["humidity"].append(newJsonData["humidity"][i])
                        trainingJSON["light"].append(newJsonData["light"][i])
                        trainingJSON["numLidarPoints"].append(newJsonData["numLidarPoints"][i])
                    
                #with open((os.path.join(root,'sameSizeAllData.json')), 'w') as fd:
                #    json.dump(newJsonData, fd)
                
    models.fitData(trainingJSON, testJSON)            
                
    #plo5 = plotter.Plotter(masterJSON)
    #plo5.plot5D(filepath)
    plo6 = plotter.Plotter(cubeJSON)
    cubeFile = filepath + "/Cubes"
    plo6.plot5D(cubeFile)
    plo7 = plotter.Plotter(handwarmerJSON)
    handwarmerFile = filepath + "/Handwarmers"
    plo7.plot5D(handwarmerFile)    
    '''allPercentages = getPercentageOfExpectedPoints(masterJSON)
    plo1 = plotter.Plotter(allPercentages)
    plo1.plot3dAreaDistance(filepath)
    plo1.plotAndSaveAllData(filepath, "Scatter_Percentages", "numLidarPoints")
    plo1.plot1DPoints()    
    plo1.plotHistograms(filepath)
    
    plo2 = plotter.Plotter(cubeJSON)
    cubeFile = filepath + "/Cubes"
    plo2.plot3dAreaDistance(cubeFile)
    plo2.plotAndSaveAllData(cubeFile, "Scatter", "numLidarPoints")
    plo2.plot1DPoints()    
    plo2.plotHistograms(cubeFile)
    cubePercentages = getPercentageOfExpectedPoints(cubeJSON)
    plo4 = plotter.Plotter(cubePercentages)
    plo4.plotAndSaveAllData(cubeFile, "Scatter_Percentages", "numLidarPoints")
    
    plo2 = plotter.Plotter(handwarmerJSON)
    handwarmerFile = filepath + "/Handwarmers"
    plo2.plot3dAreaDistance(handwarmerFile)
    plo2.plotAndSaveAllData(handwarmerFile, "Scatter", "numLidarPoints")
    plo2.plot1DPoints()    
    plo2.plotHistograms(handwarmerFile) 
    handwarmerPercentages = getPercentageOfExpectedPoints(handwarmerJSON)
    plo3 = plotter.Plotter(handwarmerPercentages)
    plo3.plotAndSaveAllData(handwarmerFile, "Scatter_Percentages", "numLidarPoints")'''
    return masterJSON

def getDataFrame(dictionary):
    dataFrame = pd.DataFrame.from_dict(dictionary)
    return dataFrame
                
def getNormalizedData(masterJSON):                
    #populate with 0 to 1 values for the masterJSON lib
    zeroToOne = {}
    for key in masterJSON:
        zeroToOne[key] = []
        maxVal = max(masterJSON[key])
        for i in range(len(masterJSON[key])):
            zeroToOne[key].append(masterJSON[key][i]/maxVal)
    return zeroToOne
    '''
    #with open((os.path.join(filepath, 'allData.json')), 'w') as fd:
    #    json.dump(masterJSON, fd)
    dataFrame = pd.DataFrame.from_dict(zeroToOne)
    #dataFrame = dataFrame.cumsum()
    #plt.figure()
    #dataFrame.plot()
    #sns.pairplot(dataFrame, hue='red')
    #sns.replot(data=masterJSON, x="area",
    g = sns.PairGrid(dataFrame)
    g.map(sns.scatterplot)'''
    
def getPercentageOfExpectedPoints(df):
    distances = [0.61,1.219,1.829]
    expected40 = [2023.5, 505.9, 225]
    expected80 = [8094.2, 2023.5, 899.4]
    expected160 = [32376.7, 8094.2, 3597.4]
    expectedHandwarmers = [78,1946.4,865]
    thisScenarioExpected = 1
    distanceIndex = 0
    
    if(len(df["green"]) == 0):
        return df
    if(len(df["area"]) == 0):
        return df
    
    #taking difference to account for potential float error as it won't likely be exact
    if abs(df["distance"][0] - 0.61) < 0.1:
        distanceIndex = 0
    elif abs(df["distance"][0] - 1.219) < 0.1:
        distanceIndex = 1
    elif abs(df["distance"][0] - 1.829) < 0.1:
        distanceIndex = 2
    else:
        print("Error! didn't recognize distance!")
        #print(df["distance"][0])
    
    #check if it's a cube
    if df["green"][0] == 255:
        if (abs(df["area"][0] - (0.04*0.04)) < 0.01):
            thisScenarioExpected = expected40[distanceIndex]
        elif (abs(df["area"][0] - (0.08*0.08)) < 0.01):
            thisScenarioExpected = expected80[distanceIndex]
        elif (abs(df["area"][0] - (0.16*0.16)) < 0.01):
            thisScenarioExpected = expected160[distanceIndex]
        else:
            print("Error, unrecognized area!")
            #print(df["area"][0])
    else: #handwarmer scenarios
        thisScenarioExpected = expectedHandwarmers[distanceIndex]
        
    points = df["numLidarPoints"]
    
    for i in range(len(points)):
        #print("Getting new point from: " + str(points[i]))
        newPoint = (points[i]/thisScenarioExpected) * 100
        points[i] = newPoint
        #print(" to " + str(newPoint))
        
    df["numLidarPoints"] = points
    return df
    
'''if __name__ == '__main__':
    if(len(sys.argv) < 2):
        print "Error! Enter base file path as an argument you'd like to traverse to process json files"
    else:
        filepath = sys.argv[1]
        if(os.path.isdir(filepath)):
            print "Error, your directory doesn't seem to exist!"
        else:
            print "Traversing directory"
            traverseDirectoriesAndProcess(filepath)'''
