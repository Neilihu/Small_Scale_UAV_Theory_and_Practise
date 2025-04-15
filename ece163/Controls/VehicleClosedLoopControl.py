'''
Author: Neili Hu (nehu@ucsc.edu)
Date: February 2025
This file is about PDI contrl for autopilot. It keep track for everything
'''
import math
import sys
import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule
import ece163.Controls.VehicleEstimator as VehicleEstimator
import ece163.Sensors.SensorsModel as SensorsModel

class VehicleClosedLoopControl():
    def __init__(self, dT=VPC.dT, rudderControlSource='SIDESLIP', useSensors=False, useEstimator=False):#done
        self.dt = dT
        self.VAM = VehicleAerodynamicsModule.VehicleAerodynamicsModel()
        self.c_gain = Controls.controlGains()
        self.trim = Inputs.controlInputs()
        self.out = Inputs.controlInputs()
        self.mode = Controls.AltitudeStates.HOLDING
        self.rollFromCourse = PIControl()
        self.rudderFromSideslip = PIControl()
        self.throttleFromAirspeed = PIControl()
        self.pitchFromAltitude = PIControl()
        self.pitchFromAirspeed = PIControl()
        self.elevatorFromPitch = PDControl()
        self.aileronFromRoll = PIDControl()
        self.sensor_true = useSensors
        self.useEstimator = useEstimator
        if self.sensor_true:
            self.sensor = SensorsModel.SensorsModel(aeroModel = self.VAM)
            if self.useEstimator:
                self.vehicleEstimator = VehicleEstimator.VehicleEstimator(dT=self.dt, sensorsModel = self.sensor)

    def reset(self):#dnoe
        self.aileronFromRoll.resetIntegrator()
        self.pitchFromAirspeed.resetIntegrator()
        self.pitchFromAltitude.resetIntegrator()
        self.throttleFromAirspeed.resetIntegrator()
        self.rudderFromSideslip.resetIntegrator()
        self.rollFromCourse.resetIntegrator()
        self.VAM.reset()
        if self.sensor_true:
            self.sensor.reset()
            if self.useEstimator:
                self.vehicleEstimator.reset()
    
    def getControlGains(self):#done
        return self.c_gain
    
    def setControlGains(self, controlGains=Controls.controlGains()):#done
        self.aileronFromRoll.setPIDGains(self.dt, controlGains.kp_roll, controlGains.kd_roll, controlGains.ki_roll, self.trim.Aileron ,VPC.minControls.Aileron, VPC.maxControls.Aileron)
        self.elevatorFromPitch.setPDGains(controlGains.kp_pitch, controlGains.kd_pitch, self.trim.Elevator, VPC.minControls.Elevator, VPC.maxControls.Elevator)
        self.pitchFromAirspeed.setPIGains(self.dt, controlGains.kp_SpeedfromElevator, controlGains.ki_SpeedfromElevator, 0, -1*math.radians(VPC.pitchAngleLimit), math.radians(VPC.pitchAngleLimit))
        self.pitchFromAltitude.setPIGains(self.dt, controlGains.kp_altitude, controlGains.ki_altitude, 0, -1*math.radians(VPC.pitchAngleLimit), math.radians(VPC.pitchAngleLimit))
        self.throttleFromAirspeed.setPIGains(self.dt, controlGains.kp_SpeedfromThrottle, controlGains.ki_SpeedfromThrottle, self.trim.Throttle, VPC.minControls.Throttle, VPC.maxControls.Throttle)
        self.rudderFromSideslip.setPIGains(self.dt, controlGains.kp_sideslip, controlGains.ki_sideslip, self.trim.Rudder, VPC.minControls.Rudder, VPC.maxControls.Rudder)
        self.rollFromCourse.setPIGains(self.dt, controlGains.kp_course, controlGains.ki_course, 0, -1*math.radians(VPC.bankAngleLimit), math.radians(VPC.bankAngleLimit))
        self.c_gain = controlGains
        
    def getVehicleState(self):#done
        return self.VAM.getVehicleState()
    
    def setVehicleState(self, state):#done
        self.VAM.setVehicleState(state)
    
    def getTrimInputs(self):#done
        return self.trim
    
    def setTrimInputs(self, trimInputs=Inputs.controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)):#done
        self.trim = trimInputs

    def getVehicleAerodynamicsModel(self):#done
        return self.VAM
    
    def getVehicleControlSurfaces(self):#done
        return self.out
    
    def UpdateControlCommands(self, referenceCommands, state):#half-done
        res = Inputs.controlInputs()
        temp = referenceCommands.commandedCourse - state.chi
        if temp >= math.pi:
            rc = self.rollFromCourse.Update(referenceCommands.commandedCourse, state.chi+2*math.pi)
        elif temp <= -1*math.pi:
            rc = self.rollFromCourse.Update(referenceCommands.commandedCourse, state.chi-2*math.pi)
        else:
            rc = self.rollFromCourse.Update(referenceCommands.commandedCourse, state.chi)
        referenceCommands.commandedRoll = rc
        res.Aileron = self.aileronFromRoll.Update(rc, state.roll, state.p)
        
        res.Rudder = self.rudderFromSideslip.Update(0, state.beta)
#state machine--------------------------------------------------------------------------
        up = referenceCommands.commandedAltitude + VPC.altitudeHoldZone
        low = referenceCommands.commandedAltitude - VPC.altitudeHoldZone
        if self.mode == Controls.AltitudeStates.DESCENDING:
            if (-1*state.pd > low) and (-1*state.pd < up):
                self.pitchFromAltitude.resetIntegrator()
                self.mode = Controls.AltitudeStates.HOLDING
        elif self.mode == Controls.AltitudeStates.HOLDING:
            if -1*state.pd > up:
                self.pitchFromAirspeed.resetIntegrator()
                self.mode = Controls.AltitudeStates.DESCENDING
            elif -1*state.pd < low:
                self.pitchFromAirspeed.resetIntegrator()
                self.mode = Controls.AltitudeStates.CLIMBING
        else:
            if (-1*state.pd > low) and (-1*state.pd < up):
                self.pitchFromAltitude.resetIntegrator()
                self.mode = Controls.AltitudeStates.HOLDING
#--------------------------------------------------------------------------------------
        if self.mode == Controls.AltitudeStates.DESCENDING:
            res.Throttle = VPC.minControls.Throttle
            temp = self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
            res.Elevator = self.elevatorFromPitch.Update(temp, state.pitch, state.q)
        elif self.mode == Controls.AltitudeStates.HOLDING:
            res.Throttle = self.throttleFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
            temp = self.pitchFromAltitude.Update(referenceCommands.commandedAltitude, -1*state.pd)
            res.Elevator = self.elevatorFromPitch.Update(temp, state.pitch, state.q)
        else:
            res.Throttle = VPC.maxControls.Throttle
            temp = self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
            res.Elevator = self.elevatorFromPitch.Update(temp, state.pitch, state.q)
        referenceCommands.commandedPitch = temp

        return res
    
    def getSensorsModel(self):#done
        res = SensorsModel.SensorsModel()
        if self.sensor_true:
            return self.sensor
        return res
    
    def getVehicleEstimator(self):#done
        res = VehicleEstimator.VehicleEstimator()
        if self.useEstimator:
            return self.vehicleEstimator
        return res
    
    def update(self, referenceCommands=Controls.referenceCommands()):#done
        if self.useEstimator:
            temp = self.UpdateControlCommands(referenceCommands,self.vehicleEstimator.getEstimatedState())
        else:
            temp = self.UpdateControlCommands(referenceCommands, self.getVehicleState())
        self.VAM.Update(temp)

        if self.sensor_true:
            self.sensor.Update()
            if self.useEstimator:
                self.vehicleEstimator.Update()
    
class PDControl():
    def __init__(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.highLimit = highLimit
        self.lowLimit = lowLimit

    def setPDGains(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.highLimit = highLimit
        self.lowLimit = lowLimit

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        e = command - current

        res = self.kp*e - self.kd*derivative + self.trim
        if res > self.highLimit:
            res = self.highLimit
        elif res < self.lowLimit:
            res = self.lowLimit

        return res
    
class PIControl():
    def __init__(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dt = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.esum = 0.
        self.pre_e = 0.
        self.highLimit = highLimit
        self.lowLimit = lowLimit

    def setPIGains(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dt = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.esum = 0.
        self.pre_e = 0.
        self.highLimit = highLimit
        self.lowLimit = lowLimit

    def Update(self, command=0.0, current=0.0):
        e = command - current
        temp = self.dt/2*(e+self.pre_e)

        res = self.kp*e + self.ki*(self.esum+temp) + self.trim
        if res > self.highLimit:
            res = self.highLimit
        elif res < self.lowLimit:
            res = self.lowLimit
        else:
            self.esum += self.dt/2*(e+self.pre_e)
        self.pre_e = e

        return res
    
    def resetIntegrator(self):
        self.esum = 0.
        self.pre_e = 0.
    
class PIDControl():
    def __init__(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dt = dT
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.trim = trim
        self.esum = 0.
        self.pre_e = 0.
        self.highLimit = highLimit
        self.lowLimit = lowLimit

    def setPIDGains(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dt = dT
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.trim = trim
        self.esum = 0.
        self.pre_e = 0.
        self.highLimit = highLimit
        self.lowLimit = lowLimit

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        e = command-current
        temp = self.dt/2*(e+self.pre_e)

        res = self.kp*e + self.ki*(self.esum+temp) - self.kd*derivative + self.trim
        
        if res > self.highLimit:
            res = self.highLimit
        elif res < self.lowLimit:
            res = self.lowLimit
        else:
            self.esum += temp

        self.pre_e = e

        return res
    
    def resetIntegrator(self):
        self.esum = 0.
        self.pre_e = 0.
