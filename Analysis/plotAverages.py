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
    ax[0,0].scatter(mm160X,mm160Y,color='red',label='160mm')
    mm80X = [0.61,1.219,1.829]
    mm80Y = [5085,1338,457]
    ax[0,0].scatter(mm80X,mm80Y,color='purple',label='80mm')
    mm40X = [0.61,1.219]
    mm40Y = [1689,309]
    ax[0,0].scatter(mm40X,mm40Y,color='blue',label='40mm')    
    ax[0,0].set_title('All Cubes')
    ax[0,0].legend(loc='upper right',shadow=True)
    
    expectedX = [0.61,1.219,1.829]
    expectedY_40 = [1961,491,218]
    ax[0,1].set_title('40mm')
    ax[0,1].scatter(mm40X,mm40Y,color='blue',label='40mm') 
    ax[0,1].scatter(expectedX,expectedY_40,color='green',label='Expected 40mm')
    ax[0,1].legend(loc='upper right',shadow=True)
    expectedY_80 = [7845,1965,873]
    ax[1,0].set_title('80mm')
    ax[1,0].scatter(mm80X,mm80Y,color='purple',label='80mm')
    ax[1,0].scatter(expectedX,expectedY_80,color='green',label='Expected 80mm')
    ax[1,0].legend(loc='upper right',shadow=True)
    expectedY_160 = [31380,7858,3491]
    ax[1,1].set_title('160mm')
    ax[1,1].scatter(mm160X,mm160Y,color='red',label='160mm')
    ax[1,1].scatter(expectedX,expectedY_160,color='green',label='Expected 160mm')
    ax[1,1].legend(loc='upper right',shadow=True)
    plt.show()
    
    fig2 = plt.figure()
    fig2.suptitle('Handwarmers # lidar point comparisons')
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
    
    ax2 = fig2.add_subplot(1,1,1,projection='3d',label='All Handwarmers')
    ax2.set(xlabel='Distance (m)', ylabel = 'Number Lidar Points')
    ax2.set_zlabel = 'Temperatures (C)'
    ax2.scatter(blackDistances, black, blackTemperatures, color='black', label='Black')
    ax2.scatter(blueDistances, blue, blueTemperatures, color='blue', label='Blue')
    ax2.scatter(rgDistances, rg, rgTemperatures, color='red', label='RoseGold')
    ax2.scatter(silverDistances, silver, silverTemperatures, color='gray', label='Silver')
    ax2.legend(loc='upper right',shadow=True)
    plt.show()
    fig3 = plt.figure()
    fig3.suptitle('Black HW lidar points compared to expected')
    ax3 = fig3.add_subplot(1,1,1,projection='3d',label='Black')
    ax3.set(xlabel='Distance (m)', ylabel = 'Number Lidar Points')
    ax3.set_zlabel = 'Temperatures (C)'
    ax3.scatter(blackDistances, black, blackTemperatures, color='black', label='Black')
    ax3.scatter(allDistances, expectedY_HW, allTemperatures, color='green', label='Ground truth')
    ax3.legend(loc='upper right',shadow=True)
    plt.show()
    fig4 = plt.figure()
    fig4.suptitle('Blue HW lidar points compared to expected')
    ax4 = fig4.add_subplot(1,1,1,projection='3d',label='Blue')
    ax4.set(xlabel='Distance (m)', ylabel = 'Number Lidar Points')
    ax4.set_zlabel = 'Temperatures (C)'    
    ax4.scatter(blueDistances, blue, blueTemperatures, color='blue', label='Blue')
    ax4.scatter(allDistances, expectedY_HW, allTemperatures, color='green', label='Ground truth')
    ax4.legend(loc='upper right',shadow=True)
    plt.show()
    fig5 = plt.figure()
    fig5.suptitle('RoseGold HW lidar points compared to expected')
    ax5 = fig5.add_subplot(1,1,1,projection='3d',label='RoseGold')
    ax5.set(xlabel='Distance (m)', ylabel = 'Number Lidar Points')
    ax5.set_zlabel = 'Temperatures (C)'
    ax5.scatter(rgDistances, rg, rgTemperatures, color='red', label='RoseGold')
    ax5.scatter(allDistances, expectedY_HW, allTemperatures, color='green', label='Ground truth')
    ax5.legend(loc='upper right',shadow=True)
    plt.show()
    fig6 = plt.figure()
    fig6.suptitle('Silver HW lidar points compared to expected')
    ax6 = fig6.add_subplot(1,1,1,projection='3d',label='Silver')
    ax6.set(xlabel='Distance (m)', ylabel = 'Number Lidar Points')
    ax6.set_zlabel = 'Temperatures (C)'
    ax6.scatter(silverDistances, silver, silverTemperatures, color='gray', label='Silver')
    ax6.scatter(allDistances, expectedY_HW, allTemperatures, color='green', label='Ground truth')
    ax6.legend(loc='upper right',shadow=True)
    ax6.set_xlabel('Number of Lidar Points')
    ax6.set_ylabel('Distance (m)')
    
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
    
    