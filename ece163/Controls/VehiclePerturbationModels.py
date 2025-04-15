'''
Author: Neili Hu(nehu@ucsc.edu)
Date: January 2025
This file calculates transfer function coefficient for Trim 
'''
import math
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath

common = VehicleAerodynamicsModel.VehicleAerodynamicsModel()

def CreateTransferFunction(trimState, trimInputs):
    p = VPC.rho
    s = VPC.S
    b = VPC.b
    c = VPC.c
    va = trimState.Va
    jy = VPC.Jyy
    m = VPC.mass
    res = Linearized.transferFunctions()

    res.Va_trim = trimState.Va
    res.alpha_trim = trimState.alpha
    res.beta_trim = trimState.beta
    res.theta_trim = trimState.pitch
    res.gamma_trim = res.theta_trim - res.alpha_trim
    res.phi_trim = trimState.roll

    res.a_phi1 = -1*p*math.fabs(va)*s*VPC.Cpp*b**2/4
    res.a_phi2 = 0.5*p*va**2*s*b*VPC.CpdeltaA
    res.a_beta1 = -1*p*va*s*VPC.CYbeta/2/m
    res.a_beta2 = p*va*s*VPC.CYdeltaR/2/m
    res.a_theta1 = -1*p*math.fabs(va)*c**2*s*VPC.CMq/4/jy
    res.a_theta2 = -1*p*va**2*c*s*VPC.CMalpha/2/jy
    res.a_theta3 = p*va**2*c*s*VPC.CMdeltaE/2/jy

    res.a_V1 = p*va*s/m*(VPC.CD0+VPC.CDalpha*res.alpha_trim+VPC.CDdeltaE*trimInputs.Elevator)-dThrust_dVa(va, trimInputs.Throttle)/m
    res.a_V2 = dThrust_dThrottle(va, trimInputs.Throttle)/m
    res.a_V3 = VPC.g0*math.cos(res.gamma_trim)
    
    return res

def dThrust_dThrottle(Va, Throttle, epsilon=0.01):
    (a,b) = common.CalculatePropForces(Va, Throttle+epsilon)
    (c,d) = common.CalculatePropForces(Va, Throttle)
    res = (a-c)/epsilon
    return res

def dThrust_dVa(Va, Throttle, epsilon=0.5):
    (a,b) = common.CalculatePropForces(Va+epsilon, Throttle)
    (c,d) = common.CalculatePropForces(Va, Throttle)
    res = (a-c)/epsilon
    return res

