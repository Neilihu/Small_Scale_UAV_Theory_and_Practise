'''
Author: Neili Hu (nehu@ucsc.edu)
Date: February 2025
This file is about estimator for autopilot. it estimate the values for each parameters
'''
import math
from ..Containers import Controls
from ..Containers import Sensors
from ..Containers import States
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleDynamicsModel as VDM
from ..Sensors import SensorsModel
from ..Utilities import MatrixMath as MM


class LowPassFilter():#done
    def __init__(self, dT=VPC.dT, cutoff=1):#done
        self.dt=dT
        self.f_cut=cutoff*2*math.pi
        self.y=0

    def reset(self):#done
        self.y=0

    def update(self, input):#done
        res = math.exp(-self.f_cut*self.dt)*self.y+(1-math.exp(-self.f_cut*self.dt))*input
        self.y=res
        return res
    
class VehicleEstimator():
    def __init__(self, dT=VPC.dT, gains=Controls.VehicleEstimatorGains(), sensorsModel=SensorsModel.SensorsModel()):#done
        self.sensor = sensorsModel
        self.gain = gains
        self.dt = dT

        self.state = States.vehicleState(pd = VPC.InitialDownPosition)
        self.state.Va = VPC.InitialSpeed
        self.filter = LowPassFilter()
        self.bias = Sensors.vehicleSensors()
        self.acent = 0
        self.first = 0
        self.tick = 0
        self.glo = 0
        self.max = VSC.GPS_rate/self.dt
        
    def reset(self):#done
        self.state.pn = 0
        self.state.pe = 0
        self.state.pd = VPC.InitialDownPosition
        self.state.u = 0
        self.state.v = 0
        self.state.w = 0
        self.state.yaw = 0
        self.state.pitch = 0
        self.state.roll = 0
        self.state.p = 0 
        self.state.q = 0
        self.state.r = 0
        self.state.Va = VPC.InitialSpeed
        self.state.R = [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
        self.filter.reset()
        self.bias.gyro_x = 0
        self.bias.gyro_y = 0
        self.bias.gyro_z = 0
        self.bias.pitot = 0
        self.bias.gps_alt = 0
        self.bias.gps_cog = 0
        self.acent = 0

    def getEstimatedState(self):#done
        return self.state
    
    def getEstimatorGains(self):#done
        return self.gain
    
    def setEstimatorGains(self, gains=Controls.VehicleEstimatorGains()):#done
        self.gain = gains

    def setEstimatedState(self, estimatedState=States.vehicleState(dcm=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])):#done
        self.state = estimatedState

    def setEstimatorBiases(self,#done
                           estimatedGyroBias=[[0], [0], [0]],
                           estimatedPitotBias=0,
                           estimatedChiBias=0,
                           estimatedAscentRate=0,
                           estimatedAltitudeGPSBias=0):
        
        self.bias = Sensors.vehicleSensors(gyro_x=estimatedGyroBias[0][0],
                                           gyro_y=estimatedGyroBias[1][0],
                                           gyro_z=estimatedGyroBias[2][0],
                                           pitot=estimatedPitotBias,
                                           gps_alt=estimatedAltitudeGPSBias,
                                           gps_cog=estimatedChiBias,)
        self.acent = estimatedAscentRate
        
    def __rexp(self, dt, state, dot):#PRIVATE
        p=state.p+dot.p*dt/2
        q=state.q+dot.q*dt/2
        r=state.r+dot.r*dt/2
        #----------------------------------------------------------
        w = MM.skew(p, q, r)
        w_norm = p*p + q*q + r*r
        w2 = MM.multiply(w,w)
        i = [[1.,0.,0.],
             [0.,1.,0.],
             [0.,0.,1.]]
        dt2 = dt**2
        #I think there should be a better way to do so-------------
        s = MM.scalarMultiply((120*dt-20*(dt**3)*w_norm+(dt**5)*(w_norm**2))/120
                                      -(dt**7)*(w_norm**3)/5040
                                      +(dt**9)*(w_norm**4)/362880
                                      -(dt**11)*(w_norm**5)/39916800, w)
        c = MM.scalarMultiply((360*dt2-30*(dt2**2)*w_norm+(dt2**3)*(w_norm**2))/720
                                      -(dt2**4)*(w_norm**3)/40320
                                      +(dt2**5)*(w_norm**4)/3628800
                                      -(dt2**6)*(w_norm**5)/479001600, w2)
        #----------------------------------------------------------
        exp = MM.add(i,MM.subtract(c,s))
        return exp

    def estimateAttitude(self, sensorData=Sensors.vehicleSensors(), estimatedState=States.vehicleState(dcm=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])):
        est_R = estimatedState.R
        est_b = [[self.bias.gyro_x],[self.bias.gyro_y],[self.bias.gyro_z]]
        i_a = MM.vectorNorm([[0],[0],[-VPC.g0]])
        i_m = MM.vectorNorm(VSC.magfield)
#-------------------------------------------------------------#if accelerometer
        b_a_den = math.hypot(sensorData.accel_x, sensorData.accel_y, sensorData.accel_z)
        if b_a_den > 0.9*VPC.g0 and b_a_den < 1.1*VPC.g0:
            b_a = MM.vectorNorm([[sensorData.accel_x],[sensorData.accel_y],[sensorData.accel_z]])
            a_error = MM.crossProduct(b_a, MM.multiply(est_R, i_a))
            bias_a = MM.scalarMultiply(-self.gain.Ki_acc, a_error)
            kp_a_err = MM.scalarMultiply(self.gain.Kp_acc, a_error)
            est_b = MM.add(est_b, MM.scalarMultiply(self.dt, bias_a))
        else:
            a_error = [[0.],[0.],[0.]]
            kp_a_err = [[0.],[0.],[0.]]
#-------------------------------------------------------------#if megnitude
        b_m = MM.vectorNorm([[sensorData.mag_x],[sensorData.mag_y],[sensorData.mag_z]])
        m_error = MM.crossProduct(b_m, MM.multiply(est_R, i_m))
        bias_m = MM.scalarMultiply(-self.gain.Ki_mag, m_error)
        kp_m_err = MM.scalarMultiply(self.gain.Kp_mag, m_error)
        est_b = MM.add(est_b, MM.scalarMultiply(self.dt, bias_m))   
#-------------------------------------------------------------
        est_w = [[sensorData.gyro_x - est_b[0][0]],
                 [sensorData.gyro_y - est_b[1][0]],
                 [sensorData.gyro_z - est_b[2][0]]]
#-------------------------------------------------------------
        temp_p = est_w[0][0] + kp_a_err[0][0] + kp_m_err[0][0]
        temp_q = est_w[1][0] + kp_a_err[1][0] + kp_m_err[1][0]
        temp_r = est_w[2][0] + kp_a_err[2][0] + kp_m_err[2][0]
        exp = self.__rexp(self.dt, States.vehicleState(p=temp_p, q=temp_q, r=temp_r), States.vehicleState())
        est_R = MM.multiply(exp, est_R)
        return est_b, est_w, est_R
    
    def estimateAltitude(self, sensorData=Sensors.vehicleSensors(), estimatedState=States.vehicleState(dcm=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])):
        h = sensorData.gps_alt#estimatedState.pd
        b_h_dot = -self.acent
        b_gps = self.bias.gps_alt
  
        #baro = (sensorData.baro-VSC.Pground)/VPC.rho/VPC.g0
        baro = sensorData.pitot/VPC.rho/VPC.g0
        h_LPF = self.filter.update(baro)*self.gain.lowPassCutoff_h
        err = h_LPF - h
        #-----------------------------------------------------
        b_h_dot = b_h_dot - self.gain.Ki_h*err*self.dt
        #-----------------------------------------------------
        input = MM.add([[0],[0],[VPC.g0]],MM.multiply(MM.transpose(estimatedState.R), [[sensorData.accel_x],[sensorData.accel_y],[sensorData.accel_z]]))[2][0]
        h_dot = input*self.dt - b_h_dot
        h = (self.gain.Kp_h*err + h_dot)*self.dt - b_gps - estimatedState.pd
        #-----------------------------------------------------
        if self.first == 1:
            b_dot_gps = -self.gain.Ki_h_gps*(sensorData.gps_alt + estimatedState.pd)
            print(b_gps, self.gain.Ki_h_gps, sensorData.gps_alt, estimatedState.pd, h, self.acent)
            b_gps = b_gps + b_dot_gps*VSC.GPS_rate

            h_dot_2 = self.gain.Kp_h_gps*(sensorData.gps_alt + estimatedState.pd)
            h = h + h_dot_2*self.dt + h*self.dt+b_gps
            self.first = 0
        else:
            self.first = 1
        return h, h_dot, b_gps
    
    def estimateAirspeed(self, sensorData=Sensors.vehicleSensors(), estimatedState=States.vehicleState(dcm=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])):#done
        #finally done
        vap = math.sqrt(2*sensorData.pitot/VPC.rho)#use pitot, don't need to use pitot-VSC.Pground
        va = estimatedState.Va
        b_va = self.bias.pitot
        va_dot = MM.add([[sensorData.accel_x],[sensorData.accel_y],[sensorData.accel_z]],
                         MM.multiply(estimatedState.R, [[0],[0],[VPC.g0]]))[0][0]
        
        b_dot_va = -self.gain.Ki_Va*(vap - va)
        b_va = b_va + b_dot_va*self.dt
        v_dot_a = va_dot - b_va + self.gain.Kp_Va*(vap - va)
        va = va + v_dot_a*self.dt

        return b_va, va
    
    def estimateCourse(self, sensorData=Sensors.vehicleSensors(), estimatedState=States.vehicleState(dcm=[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])):#done
        #finally done
        chi = estimatedState.chi
        b_chi_prev = self.bias.gps_cog
        b_chi = self.bias.gps_cog
        
        chi_dot = ((estimatedState.q*math.sin(estimatedState.roll)+estimatedState.r*math.cos(estimatedState.roll))/math.cos(estimatedState.pitch)
                    -b_chi_prev)
        chi = chi + chi_dot*self.dt

        if self.first == 1:#for gps update?
            b_dot_chi = -self.gain.Ki_chi*(sensorData.gps_cog - estimatedState.chi)
            b_chi = b_chi_prev + b_dot_chi*self.dt
            chi_dot = self.gain.Kp_chi*(sensorData.gps_cog - estimatedState.chi) - b_chi
            chi = chi + (chi_dot+self.bias.gps_cog)*self.dt
            b_chi_prev = b_chi
        self.first = 1
        return b_chi_prev, chi
    
    def Update(self):
        est_b, est_w, est_R = self.estimateAttitude(self.sensor.getSensorsNoisy(), self.state)
        self.state.R = est_R
        b_va, va = self.estimateAirspeed(self.sensor.getSensorsNoisy(), self.state)
        h, h_dot, b_gps = self.estimateAltitude(self.sensor.getSensorsNoisy(), self.state)
        b_chi_prev, chi = self.estimateCourse(self.sensor.getSensorsNoisy(), self.state)

        self.state.chi = chi
        self.state.p = est_w[0][0]
        self.state.pd = h
        self.state.q = est_w[1][0]
        self.state.r = est_w[2][0]
        self.state.Va = va

        self.setEstimatorBiases(estimatedGyroBias=est_b,
                                #estimatedPitotBias=0,  maybe needs
                                estimatedChiBias=b_chi_prev,
                                estimatedAscentRate=b_va,#?---------------------
                                estimatedAltitudeGPSBias=b_gps)