#!/usr/bin/python
import json
import os
import glob
import sys
import pandas as pd
import numpy as np
import scipy.misc
#from PIL import Image

def reconstructImage(filepath, df):
    print(filepath)
    rowFile = os.path.join(filepath,"row.txt")
    with open(rowFile) as fd:
        data = fd.readlines()
        rowCount = int(data[0].rstrip())
        #print(rowCount)
            
    pixelNumberData = df["pointNumberByFrame"]
    columnCount = int(len(pixelNumberData)/rowCount)
    
    maxVal = max(pixelNumberData)
    minVal = min(pixelNumberData)
    print("Min/max")
    #print(minVal)
    #print(maxVal)
    
    img = np.zeros((rowCount,columnCount),np.uint8)
    count = 0
    
    for c in range(columnCount):
        for r in range(rowCount):
            d = int(pixelNumberData[count])
            val = int(((255.0/float(maxVal-minVal))*(d - minVal)))
            img[r,c] = val
            #print(val)
            count = count + 1
    
    im = Image.fromarray(img)
    newFile = os.path.join(filepath,'originalImg.png')
    im.save(newFile)
    print
    #scipy.misc.imsave('originalImg.jpg', img)