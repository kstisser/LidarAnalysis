import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    fig, ax = plt.subplots(2,2)
    fig.suptitle('Cube # lidar point comparisons')
    ax[0,0].set_ylabel('Number of Lidar Points')
    ax[0,0].set_xlabel('Distance (m)')
    ax[0,1].set_ylabel('Number of Lidar Points')
    ax[0,1].set_xlabel('Distance (m)')
    ax[1,0].set_ylabel('Number of Lidar Points')
    ax[1,0].set_xlabel('Distance (m)')
    ax[1,1].set_ylabel('Number of Lidar Points')
    ax[1,1].set_xlabel('Distance (m)')    

    mm160X = [0.61,1.829]
    mm160Y = [17205,2140]
    ax[0,0].plot(mm160X,mm160Y,color='red',label='160mm')
    mm80X = [0.61,1.219,1.829]
    mm80Y = [5085,1338,475]
    ax[0,0].plot(mm80X,mm80Y,color='purple',label='80mm')
    mm40X = [0.61,1.219]
    mm40Y = [1689,309]
    ax[0,0].plot(mm40X,mm40Y,color='blue',label='40mm')    
    ax[0,0].set_title('All Cubes')
    ax[0,0].legend(loc='upper right',shadow=True)
    
    expectedX = [0.61,1.219,1.829]
    expectedY_40 = [1961,491,218]
    ax[0,1].set_title('40mm')
    ax[0,1].plot(mm40X,mm40Y,color='blue',label='40mm') 
    ax[0,1].plot(expectedX,expectedY_40,color='green',label='Expected 40mm')
    ax[0,1].legend(loc='upper right',shadow=True)
    expectedY_80 = [7845,1965,873]
    ax[1,0].set_title('80mm')
    ax[1,0].plot(mm80X,mm80Y,color='purple',label='80mm')
    ax[1,0].plot(expectedX,expectedY_80,color='green',label='Expected 80mm')
    ax[1,0].legend(loc='upper right',shadow=True)
    expectedY_160 = [31380,7858,3491]
    ax[1,1].set_title('160mm')
    ax[1,1].plot(mm160X,mm160Y,color='red',label='160mm')
    ax[1,1].plot(expectedX,expectedY_160,color='green',label='Expected 160mm')
    ax[1,1].legend(loc='upper right',shadow=True)
    plt.show()
    
    #fig2, ax2 = plt.subplots(2,3)
    fig2 = plt.figure()
    expectedY_HW = [5142,5142,5142,5142,1289.7,1289.7,1289.7,1289.7,573,573,573,573]
    allDistances = [0.61, 0.61, 0.61, 0.61, 1.219, 1.219, 1.219, 1.219,1.829,1.829,1.829,1.829]
    allTemperatures = [21,40,45,50,21,40,45,50,21,40,45,50]
    
    blackDistances = [0.61,0.61,0.61,1.219,1.219,1.219,1.829,1.829,1.829]
    blackTemperatures = [21,40,45,21,40,45,21,40,50]
    black = [2730.04,2874.2,3145.96,716.41,1526.93,1297.72,417.92,398.3,407.6]
    
    blueDistances = [0.61,0.61,0.61,1.219,1.219,1.219,1.829,1.829,1.829]
    blueTemperatures = [21,40,45,21,40,45,21,40,50]
    blue = [2604.8,3094,3538.6,765.65,1856.4,1733.4,417.9,496.6,554.35]
    
    rgDistances = [0.61,0.61,0.61,1.219,1.219,1.219,1.829,1.829,1.829]
    rgTemperatures = [21,40,45,21,40,45,21,40,50]
    rg = [3071.08,2579,2785,827.5,1761.6,1572.37,549.11,478.17,488.2]
    
    silverDistances = [0.61,0.61,0.61,1.219,1.219,1.219,1.829,1.829,1.829]
    silverTemperatures = [21,40,45,21,40,45,21,40,50]
    silver = [2724.44,2692.13,2756,788.9,1260.6,1181,637.03,517.3,407.2]    
    
    ax2 = fig.add_subplot(2,3,(1),label='All Handwarmers')
    ax2.plot(blackDistances, black, blackTemperatures, color='black', label='Black')
    ax2.plot(blueDistances, blue, blueTemperatures, color='blue', label='Blue')
    ax2.plot(rgDistances, rg, rgTemperatures, color='red', label='RoseGold')
    ax2.plot(silverDistances, silver, silverTemperatures, color='gray', label='Silver')
    ax2 = fig.add_subplot(2,3,(2),projection='3d',label='Black')
    ax2.plot(blackDistances, black, blackTemperatures, color='black', label='Black')
    ax2.plot(allDistances, expectedY_HW, allTemperatures, color='green', label='Ground truth')
    ax2 = fig.add_subplot(2,3,(3),projection='3d',label='Blue')
    ax2.plot(blueDistances, blue, blueTemperatures, color='blue', label='Blue')
    ax2.plot(allDistances, expectedY_HW, allTemperatures, color='green', label='Ground truth')
    ax2 = fig.add_subplot(2,3,(4),projection='3d',label='RoseGold')
    ax2.plot(rgDistances, rg, rgTemperatures, color='red', label='RoseGold')
    ax2.plot(allDistances, expectedY_HW, allTemperatures, color='green', label='Ground truth')
    ax2 = fig.add_subplot(2,3,(5),projection='3d',label='Silver')
    ax2.plot(silverDistances, silver, silverTemperatures, color='gray', label='Silver')
    ax2.plot(allDistances, expectedY_HW, allTemperatures, color='green', label='Ground truth')
    fig2.suptitle('Handwarmers # lidar point comparisons')
    '''ax2[0,0].set_ylabel('Number of Lidar Points')
    ax2[0,0].set_xlabel('Distance (m)')
    ax2[0,1].set_ylabel('Number of Lidar Points')
    ax2[0,1].set_xlabel('Distance (m)')
    ax2[1,0].set_ylabel('Number of Lidar Points')
    ax2[1,0].set_xlabel('Distance (m)')
    ax2[1,1].set_ylabel('Number of Lidar Points')
    ax2[1,1].set_xlabel('Distance (m)')        
    ax2[0,2].set_ylabel('Number of Lidar Points')
    ax2[0,2].set_xlabel('Distance (m)') '''      
    
    plt.show()
    
    