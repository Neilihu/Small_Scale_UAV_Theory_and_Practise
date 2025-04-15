'''
Author: Neili Hu (nehu@ucsc.edu)
Date: February 2025
This file is used to calculate gains for controlling trasnfer function.
'''
import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations

def computeGains(tuningParameters=Controls.controlTuning(), linearizedModel=Linearized.transferFunctions()):
    res = Controls.controlGains()
    res.kp_roll = tuningParameters.Wn_roll**2/linearizedModel.a_phi2
    res.kd_roll = (2*tuningParameters.Zeta_roll*tuningParameters.Wn_roll-linearizedModel.a_phi1)/linearizedModel.a_phi2
    res.ki_roll = 0.001
    res.kp_sideslip = (2*tuningParameters.Zeta_sideslip*tuningParameters.Wn_sideslip-linearizedModel.a_beta1)/linearizedModel.a_beta2
    res.ki_sideslip = tuningParameters.Wn_sideslip**2/linearizedModel.a_beta2
    res.kp_course = 2*tuningParameters.Zeta_course*tuningParameters.Wn_course*linearizedModel.Va_trim/VPC.g0
    res.ki_course = tuningParameters.Wn_course**2*linearizedModel.Va_trim/VPC.g0

    res.kp_pitch = (tuningParameters.Wn_pitch**2-linearizedModel.a_theta2)/linearizedModel.a_theta3
    res.kd_pitch = (2*tuningParameters.Zeta_pitch*tuningParameters.Wn_pitch-linearizedModel.a_theta1)/linearizedModel.a_theta3
    dc = res.kp_pitch*linearizedModel.a_theta3/(linearizedModel.a_theta2 + res.kp_pitch*linearizedModel.a_theta3)
    res.kp_altitude = 2*tuningParameters.Zeta_altitude*tuningParameters.Wn_altitude/(dc*linearizedModel.Va_trim)
    res.ki_altitude = tuningParameters.Wn_altitude**2/(dc*linearizedModel.Va_trim)
    res.kp_SpeedfromThrottle = (2*tuningParameters.Zeta_SpeedfromThrottle*tuningParameters.Wn_SpeedfromThrottle-linearizedModel.a_V1)/linearizedModel.a_V2
    res.ki_SpeedfromThrottle = tuningParameters.Wn_SpeedfromThrottle**2/linearizedModel.a_V2
    res.kp_SpeedfromElevator = (linearizedModel.a_V1-2*tuningParameters.Zeta_SpeedfromElevator*tuningParameters.Wn_SpeedfromElevator)/(dc*VPC.g0)
    res.ki_SpeedfromElevator = -1*tuningParameters.Wn_SpeedfromElevator**2/(dc*VPC.g0)

    return res

def computeTuningParameters(controlGains=Controls.controlGains(), linearizedModel=Linearized.transferFunctions()):
    res = Controls.controlTuning()
    e = Controls.controlTuning()
    if controlGains.kp_roll < 0:
        return e
    else:
        res.Wn_roll = math.sqrt(controlGains.kp_roll*linearizedModel.a_phi2)
        res.Zeta_roll = (linearizedModel.a_phi1+linearizedModel.a_phi2*controlGains.kd_roll)/(2*res.Wn_roll)
    if controlGains.ki_course < 0:
        return e
    else:
        res.Wn_course = math.sqrt(VPC.g0/linearizedModel.Va_trim*controlGains.ki_course)
        res.Zeta_course = VPC.g0/(2*res.Wn_course*linearizedModel.Va_trim)*controlGains.kp_course
    if controlGains.ki_sideslip < 0:
        return e
    else:
        res.Wn_sideslip = math.sqrt(controlGains.ki_sideslip*linearizedModel.a_beta2)
        res.Zeta_sideslip = (controlGains.kp_sideslip*linearizedModel.a_beta2+linearizedModel.a_beta1)/(2*res.Wn_sideslip)

    if linearizedModel.a_theta2 + controlGains.kp_pitch*linearizedModel.a_theta3 < 0:
        return e
    else:
        res.Wn_pitch = math.sqrt(linearizedModel.a_theta2 + controlGains.kp_pitch*linearizedModel.a_theta3)
        res.Zeta_pitch = (linearizedModel.a_theta1 + controlGains.kd_pitch*linearizedModel.a_theta3)/(2*res.Wn_pitch)
    dc = controlGains.kp_pitch*linearizedModel.a_theta3/(linearizedModel.a_theta2 + controlGains.kp_pitch*linearizedModel.a_theta3)
    if controlGains.ki_altitude < 0 and dc > 0:
        return e
    elif controlGains.ki_altitude > 0 and dc < 0:
        return e
    else:
        res.Wn_altitude = math.sqrt(dc*linearizedModel.Va_trim*controlGains.ki_altitude)
        res.Zeta_altitude = dc*linearizedModel.Va_trim*controlGains.kp_altitude/(2*res.Wn_altitude)
    if controlGains.ki_SpeedfromThrottle < 0:
        return e
    else:
        res.Wn_SpeedfromThrottle = math.sqrt(linearizedModel.a_V2*controlGains.ki_SpeedfromThrottle)
        res.Zeta_SpeedfromThrottle = (linearizedModel.a_V1+linearizedModel.a_V2*controlGains.kp_SpeedfromThrottle)/(2*res.Wn_SpeedfromThrottle)
    if controlGains.ki_SpeedfromElevator > 0 and dc > 0:
        return e
    elif controlGains.ki_SpeedfromElevator < 0 and dc < 0:
        return e
    else:
        res.Wn_SpeedfromElevator = math.sqrt(-1*dc*VPC.g0*controlGains.ki_SpeedfromElevator)
        res.Zeta_SpeedfromElevator = (linearizedModel.a_V1-dc*VPC.g0*controlGains.kp_SpeedfromElevator)/(2*res.Wn_SpeedfromElevator)
    
    return res