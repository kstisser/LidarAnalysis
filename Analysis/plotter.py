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

#supporting plotting class
class Plotter:
    def __init__(self, df):
        self.df = pd.DataFrame.from_dict(df)

    #expecting all fields to be the same length in data frame to plot them
    def plotAndSaveAllData(self, root, folderName, primeKey):
        plotsDirectory = (os.path.join(root,folderName))
        if(os.path.exists(plotsDirectory)):
            print("writing file to: " + plotsDirectory)
        else:
            os.mkdir(plotsDirectory)
        self.plotsDir = plotsDirectory
                          
        #print(self.df.columns.tolist())    
        for key in self.df:
            if key != primeKey:
                if(len(self.df[key]) > 0 and len(self.df[primeKey]) > 0):
                    print(plotsDirectory)
                    fig, ax = plt.subplots(1,1)
                    fig.suptitle('# Lidar Points vs ' + key)
                    ax.scatter(self.df[key], self.df[primeKey], marker='o')
                    #add trend line
                    z = np.polyfit(self.df[key], self.df[primeKey], 1)
                    p = np.poly1d(z)
                    plt.plot(self.df[key],p(self.df[key]),"r--")
                    ax.set_xlabel(key)
                    ax.set_ylabel(primeKey)
                    newFile = os.path.join(plotsDirectory,key)
                    newFile = newFile + ".png"          
                    plt.savefig(newFile)
                
    def plot1DPoints(self):
        fig, ax = plt.subplots(1,1)
        fig.suptitle('Lidar point variations')
        ax.plot(self.df["numLidarPoints"])
        ax.set_xlabel('Lidar number of points')
        newFile = os.path.join(self.plotsDir, 'LidarPoints.png')
        plt.savefig(newFile)    
        
    def plotHistograms(self, root):
        plotsDirectory = (os.path.join(root,"Histograms"))
        if(os.path.exists(plotsDirectory)):
            print("writing file to: " + plotsDirectory)
        else:
            os.mkdir(plotsDirectory)
                          
        print(self.df.columns.tolist())    
        for key in self.df:
            fig, ax = plt.subplots(1,1)
            fig.suptitle(key + "histogram")
            ax.hist(self.df[key])
            ax.set_xlabel(key)
            newFile = os.path.join(plotsDirectory,key)
            newFile = newFile + ".png"          
            plt.savefig(newFile)                
    #def plotStats(self):
        #will plot stats of data frame
                
    def plotMultiD(self, root):
        plotsDirectory = (os.path.join(root,"MultiD"))
        if(os.path.exists(plotsDirectory)):
            print("writing file to: " + plotsDirectory)
        else:
            os.mkdir(plotsDirectory)
                          
        fig, ax = plt.subplots(1,1)
        fig.suptitle(key + "histogram")
        ax.hist(self.df[key])
        ax.set_xlabel(key)
        newFile = os.path.join(plotsDirectory,key)
        newFile = newFile + ".png"          
        plt.savefig(newFile)      
        
    def plot3dAreaDistance(self, root):
        plotsDirectory = (os.path.join(root,"AreaDistance"))
        if(os.path.exists(plotsDirectory)):
            print("writing file to: " + plotsDirectory)
        else:
            os.mkdir(plotsDirectory)
                          
        fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
        fig.suptitle("Area distance vs % of expected points gotten")
        ax.scatter(self.df["area"],self.df["distance"],self.df["numLidarPoints"])
        ax.set_xlabel("Area")
        ax.set_ylabel("Distance")
        ax.set_zlabel("Percentage of expected Lidar points")
        newFile = os.path.join(plotsDirectory,"area.png")
        plt.savefig(newFile)         
        
    def simulate(self):        
        area = 0.18
        temperature = 21
        humidity = 35
        
        fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
        fig.suptitle("Simulated Distance Variation")
        ax.set_xlabel("Lidar Points")
        ax.set_ylabel("Distance")
        ax.set_zlabel("Area")
        error = 0
        numPoints = 0
        realPoints = []
        distances = []
        for i in range(0.25,9,0.25):
            distance = i
            realNumberOfPoints = models.getResultsFromTunedEquations(area, temperature, humidity, distance)
            perfectPoints = models.getP(area, distance)
            realPoints.append(perfectPoints)
            distances.append(i)
            error = error + (perfectPoints - realNumberOfPoints)
            numPoints = numPoints + 1
            plt.plot(realNumberOfPoints, i, area, color='red')
            
        plt.plot(realPoints, distances, np.full(distances.shape,area),color='green')
        plt.show()
            
        
    def plot5D(self, root):
        l = len(self.df["area"])
        l2 = len(self.df["distance"])
        l3 = len(self.df["temperature"])
        l4 = len(self.df["humidity"])
        l5 = len(self.df["numLidarPoints"])
        if (l == 0) or (l2 == 0) or (l3 == 0) or (l4 == 0) or (l5 == 0):
            print("Returning early")
            return     
        
        plotsDirectory = (os.path.join(root,"Final"))
        if(not os.path.exists(plotsDirectory)):
            os.mkdir(plotsDirectory)      
            
        fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
        fig.suptitle('Num Lidar Points, Size, Temperature, Humidity (color), Distance (size)')
        #5D need to display: Distance, Size, Number of Points, Temperature, Humidity
        #Plan, x=numPoints, y=humidity, z = temperature, color = size, size = distance
        xs = list(self.df["numLidarPoints"])
        minPoints = min(xs)
        maxPoints = max(xs)
        '''for i in range(len(xs)):
            xs[i] = float(xs[i] - minPoints)/float(maxPoints)'''
        cs = list(self.df["area"])
        minArea = min(cs)
        maxArea = max(cs)
        '''for i in range(len(ys)):
            ys[i] = float(ys[i] - minLight)/float(maxLight)'''
        zs = list(self.df["temperature"])
        minTemp = min(zs)
        maxTemp = max(zs)
        '''for i in range(len(zs)):
            zs[i] = float(zs[i] - minTemp)/float(maxTemp)'''
        ys = list(self.df["humidity"])
        ds = list(self.df["distance"])
        minHumid = min(ys)
        maxHumid = max(ys)
        #make range between 0 and 1
        cols = np.full(len(cs),"black")
        for i in range(len(cs)):
            if(abs(cs[i] - 0.0256) < 0.001): # 160
                cols[i] = "red"
            elif abs(cs[i] - 0.006) < 0.001: # Handwarmers
                cols[i] = "cyan"
            elif abs(cs[i] - 0.0064) < 0.001: # 80
                cols[i] = "green"
            elif abs(cs[i] - 0.0016) < 0.001: # 40
                cols[i] = "blue"
            else:
                print("Error! Don't recognize size!")
                print(cs[i])
            
        d = np.full(len(ds),"8")
        for i in range(len(ds)):
            if abs(ds[i] - 0.6096) < 0.1:
                d[i] = 'o'
            elif abs(ds[i] - 1.2192) < 0.1:
                d[i] = 'X'
            elif abs(ds[i] - 1.8288) < 0.1:
                d[i] = '*'
            else:
                print("Error! Don't recognize distance!")
                print(ds[i])
                
        data_points = [(x, y, z) for x, y, z in zip(xs, ys, zs)]
        for data, color, mar in zip(data_points, cols, d):
            x, y, z = data
            '''print("color")
            print(color)
            print("distance")
            print(mar)
            print("num lidar points")
            print(x)
            print("area")
            print(y)
            print("temperature")
            print(z)'''
            ax.scatter(x,y,z, alpha=0.05, c=color,marker=mar, edgecolors='none')
            
        z = np.polyfit(xs, ys, 1)
        p = np.poly1d(ys)
        plt.plot(xs,p(xs),"g--")
            
        ax.set_xlabel('Number Lidar Points')
        ax.set_ylabel('Humidity')
        ax.set_zlabel('Temperature (C)')
        plt.show()
        
        newFile = os.path.join(plotsDirectory,"plot5D.png")
        print("Saving new file to: ")
        print(newFile)
        plt.savefig(newFile)