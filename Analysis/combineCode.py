#!/usr/bin/python
import json
import os
import glob
import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

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
    print(filepath)
    filepath = 'C:/Users/fm873/Documents/Repos/LidarAnalysis/Results'
    for root, subdirs, files in os.walk(filepath):
        #print files
        for f in files:
            if f == 'lidarPoints.json':
                #Read in json
                with open(os.path.join(root,f)) as fd:
                    jsonData = json.load(fd)

                print(root)
                #make all fields in the file the same length so they can be added to a data frame
                tempSize = len(jsonData['temperature'])
                humidSize = len(jsonData['humidity'])
                lightSize = len(jsonData['light'])

                minVal = min(tempSize, humidSize)
                diff = abs(tempSize - humidSize)
                if minVal != 0:
                    maxLength = minVal
                else:
                    maxLength = max([tempSize,humidSize,lightSize])
                
                newTemp = {}
                if len(jsonData['temperature']) > maxLength:
                    newTemp = (jsonData['temperature'])[:maxLength]
                else:
                    newTemp = jsonData['temperature']
                    while(len(newTemp) < maxLength):
                        newTemp.append(0)

                newHumid = {}
                if len(jsonData['humidity']) > maxLength:
                    newHumid = (jsonData['humidity'])[:maxLength]
                else:
                    newHumid = jsonData['humidity']
                    while(len(newHumid) < maxLength):
                        newHumid.append(0)

                newLight = {}
                if len(jsonData['light']) > maxLength:
                    newLight = (jsonData['light'])[:maxLength]
                else:
                    newLight = (jsonData['light'])
                    while(len(newLight) < maxLength):
                        newLight.append(0)

                print("Max:")
                print(maxLength)
                #print("temp size:")
                #print(tempSize)
                #print("humid size:")
                #print(humidSize)
                #print("light size:")
                #print(lightSize)

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


                #with open((os.path.join(root,'sameSizeAllData.json')), 'w') as fd:
                #    json.dump(newJsonData, fd)

    #with open((os.path.join(filepath, 'allData.json')), 'w') as fd:
    #    json.dump(masterJSON, fd)
    dataFrame = pd.DataFrame.from_dict(masterJSON)
    #dataFrame = dataFrame.cumsum()
    #plt.figure()
    #dataFrame.plot()
    sns.pairplot(dataFrame)


if __name__ == '__main__':
    if(len(sys.argv) < 2):
        print "Error! Enter base file path as an argument you'd like to traverse to process json files"
    else:
        filepath = sys.argv[1]
        if(os.path.isdir(filepath)):
            print "Error, your directory doesn't seem to exist!"
        else:
            print "Traversing directory"
            traverseDirectoriesAndProcess(filepath)
