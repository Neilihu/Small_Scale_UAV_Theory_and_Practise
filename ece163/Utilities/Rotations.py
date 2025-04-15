import math
from . import MatrixMath
'''
Author: Neili Hu (nehu@ucsc.edu)
Date: January 9 2025
This file implements 3 rotation related funtions that can be used for the simulator
'''
def dcm2Euler(dcm_matr):
    if dcm_matr[0][1]==0.0:# optimized if-else
        yaw=0.0
    else:
        yaw = math.atan2(dcm_matr[0][1],dcm_matr[0][0])#core

    if dcm_matr[0][2]>1.0:# set simple limitation for this one
        dcm_matr[0][2]=1.0
    if dcm_matr[0][2]<-1.0:
        dcm_matr[0][2]= -1.0
    pitch = -1.0*math.asin(dcm_matr[0][2])

    if dcm_matr[1][2] == 0.0:# optimized if-else
        roll=0.0
    else:
        roll = math.atan2(dcm_matr[1][2],dcm_matr[2][2])#core

    return [yaw,pitch,roll]

def euler2DCM(yaw, pitch, roll):
    #simplified calculation
    cy=math.cos(yaw)
    sy=math.sin(yaw)
    cp=math.cos(pitch)
    sp=math.sin(pitch)
    cr=math.cos(roll)
    sr=math.sin(roll)

    res = [[cy*cp, sy*cp, -1*sp],[cy*sp*sr-sy*cr, sy*sp*sr+cy*cr, cp*sr],[cy*sp*cr+sy*sr, sy*sp*cr-cy*sr, cp*cr]]
    return res

def ned2enu(points):
    trans = [[0,1,0],[1,0,0],[0,0,-1]]

    #the return is (A*B')' = B*A' 
    return MatrixMath.multiply(points, MatrixMath.transpose(trans))

