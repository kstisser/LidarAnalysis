import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    fig.suptitle('Distance, Temperature, and % of expected Lidar Points')
    ax.set_zlabel('% of Expected Lidar Points')
    ax.set_ylabel('Distance (m)') 
    ax.set_xlabel('Temperatures (C)')
   
    Distances = [0.61,1.219,1.829]
    expectedY_40 = [1961,491,218]
    
    distances = [0.61,0.61,0.61,0.61,0.61,0.61,0.61,0.61,0.61,0.61,0.61,0.61,1.219,1.219,1.219,1.219,1.219,1.219,1.219,1.219,1.219,1.219,1.219,1.219,1.829,1.829,1.829,1.829,1.829,1.829,1.829,1.829,1.829,1.829,1.829,1.829]
    temperatures = [21,40,45,21,40,45,21,40,45,21,40,45,21,40,45,21,40,45,21,40,45,21,40,45,21,40,50,21,40,50,21,40,50,21,40,50]
    percentageOfExpectedPoints = [60,50,54,53,56,61,50,60,68,53,52,54,64,137,122,55,118,100,59,144,134,61,98,92,96,83,85,72,69,71,72,87,97,111,90,71]
    colors=[1,1,1,2,2,2,3,3,3,4,4,4,1,1,1,2,2,2,3,3,3,4,4,4,1,1,1,2,2,2,3,3,3,4,4,4,1,1,1,2,2,2,3,3,3,4,4,4]
    ax.scatter(temperatures, distances, percentageOfExpectedPoints) 

    plt.show()