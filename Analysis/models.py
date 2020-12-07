import sys
import pandas as pd
import numpy as np
import math
from scipy.optimize import curve_fit
from scipy.optimize import least_squares

def fitData(training, test):
    print("Expected num of points")
    print(training['numLidarPoints'][0])
    X = (training['area'], training['temperature'], training['humidity'], training['distance'])
    
    if len(training['area']) == 0 or len(training['temperature']) == 0 or len(training['humidity']) == 0 or len(training['distance']) == 0:
        return   

    c = 1
    b = 1
    a = 1

    Y = LinearSeparatedModel(X, a,b,c)
    popt, pcov = curve_fit(LinearSeparatedModel,X, Y, p0=(a,b,c))
    if(c != popt[0] or a != popt[1]):
        print("Got one!")
        print("Optimized: " + str(c) + ", " + str(a))

    print(popt)
    print("Finished")

'''def fitData(training, test):
    X = (training['area'], training['temperature'], training['humidity'], training['distance'])
    
    if len(training['area']) == 0 or len(training['temperature']) == 0 or len(training['humidity']) == 0 or len(training['distance']) == 0:
        return
    
        for i in training["area"]:
            if (math.inf in i) or (math.nan in i):
                return
        for i in training["distance"]:
            if (math.inf in i) or (math.nan in i):
                return
        for i in training["temperature"]: 
            if (math.inf in i) or (math.nan in i):
                return
        for i in training["humidity"]:
            if (math.inf in i) or (math.nan in i):
                return
        for i in training["numLidarPoints"]:
            if (math.inf in i) or (math.nan in i):
                return     
    
    for i in range(1,10000):
        for j in range(1,10000):
            if i != 0 and j != 0:
                c = i
                a = j

                Y = LinearCombinedModel(X, c, a)
                popt, pcov = curve_fit(LinearCombinedModel,X, Y, p0=(c, a))
                if(c != popt[0] or a != popt[1]):
                    print("Got one!")
                    print("Optimized: " + str(c) + ", " + str(a))
    #res = least_squares(LinearCombinedModel, X,args=(c,a))
    #print(res)
                #print(popt)
    print("Finished")'''

def getP(S, D):
    return (470600 * S/(D * D))

def LinearCombinedModel(X, c, a):
    S, T, H, D = X
    D = np.array(D,dtype="float64")
    S = np.array(S,dtype="float64")
    T = np.array(T,dtype="float64")
    H = np.array(H,dtype="float64")
    P = getP(S,D)
    shape = S.shape
    c = np.full(shape,c,dtype="float64")
    a = np.full(shape,a,dtype="float64")
    
    '''print("Distance count")
    print(np.count_nonzero(np.isnan(D)))
    print(np.count_nonzero(np.isinf(D)))
    print("Size")
    print(np.count_nonzero(np.isnan(S)))
    print(np.count_nonzero(np.isinf(S)))
    print("Temp")
    print(np.count_nonzero(np.isnan(T)))
    print(np.count_nonzero(np.isinf(T)))
    print("humidity")
    print(np.count_nonzero(np.isnan(H)))
    print(np.count_nonzero(np.isinf(H)))
    print("Pixels")
    print(np.count_nonzero(np.isnan(P)))
    print(np.count_nonzero(np.isinf(P))) '''  
    '''print("S")
    print(S[0])
    print("T")
    print(T[0])
    print("H")
    print(H[0])
    print("D")
    print(D[0])'''
    ans = (P - (P * c * S * T * H)/(a * D))
    #print(ans)
    return ans

def ExpCombinedModel(X,a, b, c):
    S, T, H, D = X
    D = np.array(D,dtype="float64")
    S = np.array(S,dtype="float64")
    T = np.array(T,dtype="float64")
    H = np.array(H,dtype="float64")
    P = getP(S,D)
    shape = S.shape
    a = np.full(shape,a,dtype="float64")
    b = np.full(shape,b,dtype="float64")
    c = np.full(shape,c,dtype="float64")
    return (P - (math.exp(b * S) * (P * c * T * H))/(a * D))

def LinearSeparatedModel(X,a, b, c):
    S, T, H, D = X
    D = np.array(D,dtype="float64")
    S = np.array(S,dtype="float64")
    T = np.array(T,dtype="float64")
    H = np.array(H,dtype="float64")
    P = getP(S,D)
    shape = S.shape
    a = np.full(shape,a,dtype="float64")
    b = np.full(shape,b,dtype="float64")
    c = np.full(shape,c,dtype="float64")
    print("S")
    print(S[0])
    print("T")
    print(T[0])
    print("H")
    print(H[0])
    print("D")
    print(D[0])
    return (P - (b * S * P) - (c * T * H * P) - P/(a * D))

def ExpSeparatedModel(X,a,b,c):
    S, T, H, D = X
    D = np.array(D,dtype="float64")
    S = np.array(S,dtype="float64")
    T = np.array(T,dtype="float64")
    H = np.array(H,dtype="float64")
    P = getP(S,D)
    shape = S.shape
    a = np.full(shape,a,dtype="float64")
    b = np.full(shape,b,dtype="float64")
    c = np.full(shape,c,dtype="float64")
    return (P - (math.exp(b * S) * P) - (c * T * H * P) - P/(a * D))

def getResultFromTunedEquation(S,T,H,D):
    '''S = df['area']
    T = df['temperature']
    H = df['humidity']
    D = df['distance']
    D = np.array(D,dtype="float64")
    S = np.array(S,dtype="float64")
    T = np.array(T,dtype="float64")
    H = np.array(H,dtype="float64") '''   
    P_D = (470600 * S/(D * D)) * (1 - (41.8*S) - (0.003 * T * H) - 1/(4.08*D))
    #P_D = (470600 * S/(D * D)) * (1 - (43.3*S) - (0.00008 * T * H) - 1/(5.26*D))
    return P_D