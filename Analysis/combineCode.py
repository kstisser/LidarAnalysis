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
                print(np.mean(jsonData['pointNumberByFrame']))
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

                print("Max:")
                print(maxLength)
                print("temp size:")
                print(len(newTemp))
                print("humid size:")
                print(len(newHumid))
                print("light size:")
                print(len(newLight))
                print("point num size:")
                print(len(newPoints))

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
                
                try:
                    plo = plotter.Plotter(newJsonData)
                    plo.plotAndSaveAllData(root, "Scatter_NoZero", "numLidarPoints")
                    plo.plot1DPoints()
                    plo.plotHistograms(root)
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


                #with open((os.path.join(root,'sameSizeAllData.json')), 'w') as fd:
                #    json.dump(newJsonData, fd)
                
    plo2 = plotter.Plotter(masterJSON)
    plo2.plotAndSaveAllData(filepath, "Scatter_NoZero", "numLidarPoints")
    plo2.plot1DPoints()    
    plo2.plotHistograms(filepath)
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
