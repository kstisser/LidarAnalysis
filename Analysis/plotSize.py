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
    fig, ax = plt.subplots()
    fig.suptitle('Size to Expected % of Lidar Points')
    ax.set_ylabel('% of Expected Lidar Points') 
    ax.set_xlabel('Sizes (m^2)')
   
    Distances = [0.61,1.219,1.829]
    expectedY_40 = [1961,491,218]
    
    sizes = [1.6, 6.4, 25.6]
    percentages = [74.5,62.3,57.5]
    ax.plot(sizes, percentages) 

    plt.show()