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
                          
        print(self.df.columns.tolist())    
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