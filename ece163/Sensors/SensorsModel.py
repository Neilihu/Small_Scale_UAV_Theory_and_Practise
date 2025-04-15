'''
Author:Neili Hu (nehu@ucsc.edu)
Date: Feburary 2025
This file implement sensor state that used to simulate real working sensors
'''
import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Utilities import MatrixMath
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel

class SensorsModel():
    def __init__(self,#done
                 aeroModel = VehicleAerodynamicsModel.VehicleAerodynamicsModel(),
                 taugyro = VSC.gyro_tau,
                 etagyro = VSC.gyro_eta,
                 tauGPS = VSC.GPS_tau,
                 etaGPSHorizontal = VSC.GPS_etaHorizontal, 
                 etaGPSVertical = VSC.GPS_etaVertical,
                 gpsUpdateHz = VSC.GPS_rate):
        
        self.aeroModel = aeroModel
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsBiases = self.initializeBiases()
        self.sensorSigmas = self.initializeSigmas()
        self.sensorsNoisy = Sensors.vehicleSensors()
        self.dT = VPC.dT
        self.gpsTickUpdate = gpsUpdateHz/self.dT
        self.updateTicks = 1
        self.first = 0
        self.c = 0
        self.glo = 0
        self.gyo = GaussMarkov(self.dT, taugyro, etagyro)
        self.gps_h = GaussMarkov(self.dT, tauGPS, etaGPSHorizontal)
        self.gps_v = GaussMarkov(self.dT, tauGPS, etaGPSVertical)

    def initializeBiases(self, #done
                         gyroBias = VSC.gyro_bias,
                         accelBias = VSC.accel_bias, 
                         magBias = VSC.mag_bias,
                         baroBias = VSC.baro_bias,
                         pitotBias = VSC.pitot_bias):
        
        res = Sensors.vehicleSensors(gyro_x = random.uniform(-gyroBias, gyroBias),
                                     gyro_y = random.uniform(-gyroBias, gyroBias),
                                     gyro_z = random.uniform(-gyroBias, gyroBias),
                                     accel_x = random.uniform(-accelBias, accelBias),
                                     accel_y = random.uniform(-accelBias, accelBias),
                                     accel_z = random.uniform(-accelBias, accelBias),
                                     mag_x = random.uniform(-magBias, magBias),
                                     mag_y = random.uniform(-magBias, magBias),
                                     mag_z = random.uniform(-magBias, magBias),
                                     baro = random.uniform(-baroBias, baroBias),
                                     pitot = random.uniform(-pitotBias, pitotBias))
        return res
    
    def initializeSigmas(self,#done
                         gyroSigma = VSC.gyro_sigma,
                         accelSigma = VSC.accel_sigma,
                         magSigma = VSC.mag_sigma,
                         baroSigma = VSC.baro_sigma,
                         pitotSigma = VSC.pitot_sigma,
                         gpsSigmaHorizontal = VSC.GPS_sigmaHorizontal,
                         gpsSigmaVertical = VSC.GPS_sigmaVertical,
                         gpsSigmaSOG = VSC.GPS_sigmaSOG,
                         gpsSigmaCOG = VSC.GPS_sigmaCOG):
        
        res = Sensors.vehicleSensors(gyro_x = gyroSigma, gyro_y = gyroSigma, gyro_z = gyroSigma,
                                     accel_x = accelSigma, accel_y = accelSigma, accel_z = accelSigma,
                                     mag_x = magSigma, mag_y = magSigma, mag_z = magSigma,
                                     baro = baroSigma, pitot = pitotSigma,
                                     gps_n = gpsSigmaHorizontal, gps_e = gpsSigmaHorizontal,gps_alt = gpsSigmaVertical,
                                     gps_sog = gpsSigmaSOG, gps_cog = gpsSigmaCOG)
        return res
    
    def updateGPSTrue(self, state, dot):#done
        n = state.pn
        e = state.pe
        d = -state.pd
        sog = math.hypot(dot.pn, dot.pe)
        cog = math.atan2(dot.pe, dot.pn)
        
        return n,e,d,sog,cog
    
    def updateAccelsTrue(self, state, dot):#done
        g = VPC.g0
        x = dot.u + state.q*state.w - state.r*state.v + g*math.sin(state.pitch)
        y = dot.v + state.r*state.u - state.p*state.w - g*math.cos(state.pitch)*math.sin(state.roll)
        z = dot.w + state.p*state.v - state.q*state.u - g*math.cos(state.pitch)*math.cos(state.roll)

        return x,y,z
    
    def updateMagsTrue(self, state):#done
        [[x],[y],[z]] = MatrixMath.multiply(state.R, VSC.magfield)

        return x,y,z
    
    def updateGyrosTrue(self, state):#done
        x = state.p
        y = state.q
        z = state.r

        return x,y,z
    
    def updatePressureSensorsTrue(self, state):#done
        baro = VSC.Pground + VPC.rho*VPC.g0*state.pd
        pitot = VPC.rho*state.Va**2/2

        return baro, pitot
    
    def updateSensorsTrue(self, prevTrueSensors, state, dot):#done
        res = Sensors.vehicleSensors()
        res.accel_x, res.accel_y, res.accel_z = self.updateAccelsTrue(state, dot)
        res.gyro_x, res.gyro_y, res.gyro_z = self.updateGyrosTrue(state)
        res.mag_x, res.mag_y, res.mag_z = self.updateMagsTrue(state)
        res.baro, res.pitot = self.updatePressureSensorsTrue(state)

        '''
        This part is little bit triky. The first update only use 49 ticks and the
        rest update use 50 ticks
        '''
        if self.first == 0:#first update
            if self.glo == 0:
                self.updateTicks += 1
                self.first = 1
            res.gps_n, res.gps_e, res.gps_alt, res.gps_sog, res.gps_cog = self.updateGPSTrue(state, dot)

        elif self.gpsTickUpdate != self.updateTicks:#not update
            if self.glo == 0:
                self.updateTicks += 1

            res.gps_n = prevTrueSensors.gps_n
            res.gps_e = prevTrueSensors.gps_e
            res.gps_alt = prevTrueSensors.gps_alt
            res.gps_sog = prevTrueSensors.gps_sog
            res.gps_cog = prevTrueSensors.gps_cog

        else:#update
            if self.glo == 0:
                self.updateTicks = 1

            res.gps_n, res.gps_e, res.gps_alt, res.gps_sog, res.gps_cog = self.updateGPSTrue(state, dot)
         

        return res
    
    def __helper(self, a):
        return random.gauss(0,a**2)

    def updateSensorsNoisy(self,
                           trueSensors=Sensors.vehicleSensors(),
                           noisySensors=Sensors.vehicleSensors(),
                           sensorBiases=Sensors.vehicleSensors(),
                           sensorSigmas=Sensors.vehicleSensors()):
        res = Sensors.vehicleSensors()

        gy_markove = self.gyo.update()
        res.gyro_x = sensorBiases.gyro_x + self.__helper(sensorSigmas.gyro_x) + gy_markove
        res.gyro_y = trueSensors.gyro_y + sensorBiases.gyro_y + self.__helper(sensorSigmas.gyro_y) + gy_markove
        res.gyro_z = trueSensors.gyro_z + sensorBiases.gyro_z + self.__helper(sensorSigmas.gyro_z) + gy_markove
        res.accel_x = trueSensors.accel_x + self.__helper(sensorSigmas.accel_x)
        res.accel_y = trueSensors.accel_y + self.__helper(sensorSigmas.accel_y)
        res.accel_z = trueSensors.accel_z + self.__helper(sensorSigmas.accel_z)
        res.mag_x = trueSensors.mag_x + sensorBiases.mag_x + self.__helper(sensorSigmas.mag_x)
        res.mag_y = trueSensors.mag_y + sensorBiases.mag_y + self.__helper(sensorSigmas.mag_y)
        res.mag_z = trueSensors.mag_z + sensorBiases.mag_z + self.__helper(sensorSigmas.mag_z)
        res.baro = trueSensors.baro + sensorBiases.baro + self.__helper(sensorSigmas.baro)
        res.pitot = trueSensors.pitot + sensorBiases.pitot + self.__helper(sensorSigmas.pitot)

        if trueSensors.gps_sog != 0.0:
            white = VPC.InitialSpeed/trueSensors.gps_sog
        else:
            white = 1
        '''
        Im highly doubt this function implemented wrong because I only test once and it passed
        '''
        if self.first == 0:#first update
            if self.glo == 0:
                self.updateTicks += 1
                self.first = 1
            h = self.gps_h.update()
            v = self.gps_v.update()
            res.gps_n = trueSensors.gps_n + sensorBiases.gps_n + sensorSigmas.gps_n + h
            res.gps_e = trueSensors.gps_e + sensorBiases.gps_e + sensorSigmas.gps_e + h
            res.gps_alt = trueSensors.gps_alt + sensorBiases.gps_alt + sensorSigmas.gps_alt + v
            res.gps_sog = trueSensors.gps_sog + sensorBiases.gps_sog + self.__helper(sensorSigmas.gps_sog)
            res.gps_cog = trueSensors.gps_cog + sensorBiases.gps_cog + self.__helper(sensorSigmas.gps_cog)*white
            res.gps_cog = min(max(res.gps_cog, -math.pi), math.pi)

        elif self.gpsTickUpdate != self.updateTicks:#not update
            if self.glo == 0:
                self.updateTicks+=1

            res.gps_n = noisySensors.gps_n
            res.gps_e = noisySensors.gps_e
            res.gps_alt = noisySensors.gps_alt
            res.gps_sog = noisySensors.gps_sog
            res.gps_cog = noisySensors.gps_cog
        else:#update
            self.updateTicks = 1
            h = self.gps_h.update()
            v = self.gps_v.update()
            res.gps_n = trueSensors.gps_n + sensorBiases.gps_n + sensorSigmas.gps_n + h
            res.gps_e = trueSensors.gps_e + sensorBiases.gps_e + sensorSigmas.gps_e + h
            res.gps_alt = trueSensors.gps_alt + sensorBiases.gps_alt + sensorSigmas.gps_alt + v
            res.gps_sog = trueSensors.gps_sog + sensorBiases.gps_sog + self.__helper(sensorSigmas.gps_sog)
            res.gps_cog = trueSensors.gps_cog + sensorBiases.gps_cog + self.__helper(sensorSigmas.gps_cog)*white
            res.gps_cog = min(max(res.gps_cog, -math.pi), math.pi)

        return res
    
    def Update(self):
        self.glo = 1
        state = self.aeroModel.dnamc.state
        dot = self.aeroModel.dnamc.dot
        self.sensorsTrue = self.updateSensorsTrue(self.sensorsTrue,state,dot)
        self.sensorsNoisy = self.updateSensorsNoisy(self.sensorsTrue, self.sensorsNoisy, self.sensorsBiases, self.sensorSigmas)
        self.updateTicks+=1
        self.first = 1

    def setSensorsTrue(self, sensorsTrue = Sensors.vehicleSensors()):#done
        self.sensorsTrue = sensorsTrue

    def getSensorsTrue(self):#done
        return self.sensorsTrue
    
    def setSensorsNoisy(self, sensorsNoizy = Sensors.vehicleSensors()):#done
        self.sensorsNoisy = sensorsNoizy

    def getSensorsNoisy(self):#done
        return self.sensorsNoisy
    
    def reset(self):#done
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsBiases = self.initializeBiases()
        self.sensorSigmas = self.initializeSigmas()
        self.sensorsNoisy = Sensors.vehicleSensors()
        self.updateTicks = 1
        self.first = 0
        self.glo=0
        self.gyo.reset()
        self.gps_h.reset()
        self.gps_v.reset()

class GaussMarkov():#done
    def __init__(self, dT=VPC.dT, tau=1e6, eta=0.0):#done
        self.tau = tau
        self.eta = eta
        self.v = 0
        self.dT = dT

    def reset(self):#done
        self.v = 0

    def update(self, vnoise=None):#done
        res = 0

        res = math.exp(-1*self.dT/self.tau)*self.v
        res += random.gauss(0, self.eta) if vnoise is None else vnoise
        self.v = res
        
        return res

class GaussMarkovXYZ():#done
    def __init__(self, dT=0.01, tauX=1000000.0, etaX=0.0, tauY=None, etaY=None, tauZ=None, etaZ=None):#done
        self.dT = dT
        self.x = GaussMarkov(0.01, tauX, etaX)
        if (tauY is None) and (tauZ is None):
            self.y = GaussMarkov(0.01, tauX, etaX)
            self.z = GaussMarkov(0.01, tauX, etaX)
        elif (tauY is not None) and (tauZ is None):
            self.y = GaussMarkov(0.01, tauY, etaY)
            self.z = GaussMarkov(0.01, tauY, etaY)
        elif (tauY is None) and (tauZ is not None):
            self.y = GaussMarkov(0.01, tauZ, etaZ)
            self.z = GaussMarkov(0.01, tauZ, etaZ)
        else:
            self.y = GaussMarkov(0.01, tauY, etaY)
            self.z = GaussMarkov(0.01, tauZ, etaZ)

    def reset(self):#done
        self.x.reset()
        self.y.reset()
        self.z.reset()

    def update(self, vXnoise=None, vYnoise=None, vZnoise=None):#done
        x = self.x.update(vXnoise)
        y = self.y.update(vYnoise)
        z = self.z.update(vZnoise)
        return x,y,z
