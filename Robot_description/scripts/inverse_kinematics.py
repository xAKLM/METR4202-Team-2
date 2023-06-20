import numpy as np
import modern_robotics as mr

"""
T is the desired end position
thetalist0 initial guess
returns the thetalist of the desired end position
"""
def analytical(T):
    L1 = 0.10
    L2 = 0.11827
    radius = 0.0225
    
    (rotation,position) = mr.TransToRp(T)

    x = position[0]
    y = position[1]
    z = position[2]

    betaArg = (x**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2) 
    gammaArg = (L1**2 +  x**2 + z**2 - L2**2) / (2 * L1 * np.sqrt(x**2 + z**2))
    hype = x**2 + z**2
    length = L1 + L2
    print(hype)
    print(length)
    print(betaArg)
    print(gammaArg)
    if (-1 > betaArg or betaArg > 1 ):

        print("cannot reach")
        return

    test = np.array([[1, 0, 0, 0.21827],
                  [0, 1, 0, 0.07138],
                  [0, 0, 1,       0],
                  [0, 0, 0,       1]])


    beta = np.arccos(-betaArg) 
    gamma = np.arccos(gammaArg)

    theta_1 = np.arctan2(x,z) - gamma
    theta_2 = np.pi - beta
    theta_3 = 0 #-z/radius
    theta_4 = 0
    

    return np.array([theta_1, theta_2, theta_3, theta_4])



    

def invKin(T, thetalist0):
    Slist = np.array([[0, 1, 0, 0,  0,       0],
                      [0, 1, 0, 0,  0,     0.1],
                      [0, 0, 0, 0, -1,       0],
                      [0, 1, 0, 0,  0, 0.21827]]).T

    M = np.array([[1, 0, 0, 0.21827],
                  [0, 1, 0, 0.07138],
                  [0, 0, 1,       0],
                  [0, 0, 0,       1]])

    eomg = 1
    ev = 1

    thetalist = thetalist0
    success = False
    
    [thetalist, success] = mr.IKinSpace(Slist, M, T, thetalist, eomg, ev)

    return (thetalist, success)
    
    
