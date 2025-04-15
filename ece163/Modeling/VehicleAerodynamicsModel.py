'''
Author: Neili Hu (nehu@ucsc.edu)
Date: Janurary 2025
This file implements some aero related functions. 
vehicle dynamic model is required before implementation
'''
import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

class VehicleAerodynamicsModel:
    def __init__(self, initialSpeed=25.0, initialHeight=-100.0):#done
        self.speed = initialSpeed
        self.height = initialHeight
        self.wind = WindModel.WindModel()
        self.dnamc = VehicleDynamicsModel.VehicleDynamicsModel()
        self.dnamc.state.u = self.speed
        self.dnamc.state.pd = self.height
        self.dt = VPC.dT

    def CalculateAirspeed(self, state, wind):#done
        ws = math.hypot(wind.Wn, wind.We, wind.Wd)
        xw = math.atan2(wind.We, wind.Wn)
        yw = 0. if ws==0. else -1*math.asin(wind.Wd/ws)

        r = [[math.cos(xw)*math.cos(yw), -1*math.sin(xw), math.cos(xw)*math.sin(yw)],
             [math.sin(xw)*math.cos(yw), math.cos(xw), math.sin(xw)*math.sin(yw)],
             [  -1*math.sin(yw),             0.             , math.cos(yw)]]
        #originally, I directly modified wind.wu/wv/ww. This version didn't.
        #It also can pass test. Hence, I leave it here.
        [[tu],[tv],[tw]] = MatrixMath.multiply(r, [[wind.Wu],[wind.Wv],[wind.Ww]])
        temp = MatrixMath.add([[wind.Wn],[wind.We],[wind.Wd]], [[tu],[tv],[tw]])
        '''
        [[wind.Wu],[wind.Wv],[wind.Ww]] = MatrixMath.multiply(r, [[wind.Wu],[wind.Wv],[wind.Ww]])
        temp = MatrixMath.add([[wind.Wn],[wind.We],[wind.Wd]], [[wind.Wu],[wind.Wv],[wind.Ww]])
        '''
        Va = MatrixMath.multiply(Rotations.euler2DCM(state.yaw, state.pitch, state.roll), temp)
        temp = MatrixMath.subtract([[state.u],[state.v],[state.w]], Va)
        Va = math.hypot(temp[0][0],temp[1][0],temp[2][0])

        alpha = math.atan2(temp[2][0], temp[0][0])
        beta = 0. if Va == 0. else math.asin(temp[1][0]/Va)
        return Va, alpha, beta
    
    def CalculateCoeff_alpha(self, alpha):#done #PRIVATE
        a0 = VPC.alpha0
        one = VPC.M*(a0-alpha)
        two = VPC.M*(a0+alpha)
        bleed = (1+math.exp(one)+math.exp(two))/((1+math.exp(one))*(1+math.exp(two)))
        
        cl = VPC.CL0 + VPC.CLalpha*alpha
        cla = (1-bleed)*cl + bleed*2*math.sin(alpha)*math.cos(alpha)

        cd = VPC.CDp + (cl**2)/(math.pi*VPC.AR*VPC.e)
        cda = (1-bleed)*cd + bleed*(1-math.cos(2*alpha))

        cma = VPC.CM0 + VPC.CMalpha*alpha

        return cla, cda, cma
    
    def CalculatePropForces(self, Va, Throttle):#done #PRIVATE
        d = VPC.D_prop
        p = VPC.rho
        pi = math.pi
        a = p*(d**5)*VPC.C_Q0/(4*(pi**2))
        b = p*(d**4)*Va*VPC.C_Q1/(2*pi) + (VPC.KQ**2)/VPC.R_motor
        c = p*(d**3)*(Va**2)*VPC.C_Q2 - VPC.KQ*VPC.V_max*Throttle/VPC.R_motor + VPC.KQ*VPC.i0

        if (((b**2) - 4*a*c) < 0):
            o = 100.0
        elif (a == 0):
            o = 0.
        else:
            o = (-1*b + math.sqrt((b**2)-4*a*c))/(2*a)
        
        j = 0 if (o == 0) else 2*pi*Va/(o*d)

        ct = VPC.C_T0 + VPC.C_T1*j + VPC.C_T2*(j**2)
        cq = VPC.C_Q0 + VPC.C_Q1*j + VPC.C_Q2*(j**2)

        fx = p*(o**2)*(d**4)*ct/(4*(pi**2))
        mx = -1*p*(o**2)*(d**5)*cq/(4*(pi**2))

        return (fx,mx)
    
    def Update(self, controls):#done
        state = self.dnamc.state
        forces = self.updateForces(state, controls, self.wind.wind)
        self.dnamc.Update(forces)

    def aeroForces(self, state):#done #PRIVATE
        res = Inputs.forcesMoments()
        va = state.Va
        alpha = state.alpha
        beta = state.beta
        cos = VPC.rho*(va**2)*VPC.S/2
        c = VPC.c
        b = VPC.b
        cla, cda, cma = self.CalculateCoeff_alpha(alpha)
        temp = [[math.cos(alpha), -1*math.sin(alpha)],[math.sin(alpha), math.cos(alpha)]]

        if (va == 0):
            res.Fx = cos*cla
            res.Fy = cos*cda
            [[res.Fx],[res.Fz]] = MatrixMath.multiply(temp, [[-1*res.Fy],[-1*res.Fx]])
            res.Fy = cos*(VPC.CY0 + VPC.CYbeta*beta)

            res.Mx = cos*b*(VPC.Cl0 + VPC.Clbeta*beta)
            res.My = cos*c*(cma)
            res.Mz = cos*b*(VPC.Cn0 + VPC.Cnbeta*beta)
        else:
            res.Fx = cos*(cla + VPC.CLq*c*state.q/(2*va))
            res.Fy = cos*(cda + VPC.CDq*c*state.q/(2*va))
            [[res.Fx],[res.Fz]] = MatrixMath.multiply(temp, [[-1*res.Fy],[-1*res.Fx]])
            res.Fy = cos*(VPC.CY0 + VPC.CYbeta*beta + VPC.CYp*b*state.p/(2*va) + VPC.CYr*b*state.r/(2*va))

            res.Mx = cos*b*(VPC.Cl0 + VPC.Clbeta*beta + VPC.Clp*b*state.p/(2*va) + VPC.Clr*b*state.r/(2*va))
            res.My = cos*c*(cma + VPC.CMq*c*state.q/(2*va))
            res.Mz = cos*b*(VPC.Cn0 + VPC.Cnbeta*beta + VPC.Cnp*b*state.p/(2*va) + VPC.Cnr*b*state.r/(2*va))

        return res

    def controlForces(self, state, controls):#done #PRIVATE
        res = Inputs.forcesMoments()
        temp =  [[math.cos(state.alpha), -1*math.sin(state.alpha)],[math.sin(state.alpha), math.cos(state.alpha)]]
        cos = VPC.rho*(state.Va**2)*VPC.S/2
        Aileron = controls.Aileron
        Rudder = controls.Rudder
        Elevator = controls.Elevator

        (f2, m2) = self.CalculatePropForces(state.Va, controls.Throttle)
        x = cos*VPC.CLdeltaE*Elevator
        z = cos*VPC.CDdeltaE*Elevator
        [[x],[z]] = MatrixMath.multiply(temp, [[-1*z],[-1*x]])
        y = cos*(VPC.CYdeltaA*Aileron + VPC.CYdeltaR*Rudder)

        l = cos*VPC.b*(VPC.CldeltaA*Aileron + VPC.CldeltaR*Rudder)
        m = cos*VPC.c*VPC.CMdeltaE*Elevator
        n = cos*VPC.b*(VPC.CndeltaA*Aileron + VPC.CndeltaR*Rudder)

        res.Fx += x + f2
        res.Fy += y
        res.Fz += z

        res.Mx += l + m2
        res.My += m
        res.Mz += n

        return res
    
    def getVehicleDynamicsModel(self):#done
        return self.dnamc

    def getVehicleState(self):#done
        return self.dnamc.state

    def getWindModel(self):#done
        return self.wind
    
    def gravityForces(self, state):#done # PRIVATE
        res = Inputs.forcesMoments()

        [[res.Fx],[res.Fy],[res.Fz]]=MatrixMath.multiply(MatrixMath.scalarMultiply(VPC.mass,state.R), [[0],[0],[VPC.g0]])
        
        return res
    
    def reset(self):#done
        self.wind = WindModel.WindModel()
        self.dnamc = VehicleDynamicsModel.VehicleDynamicsModel()
        self.dnamc.state.u = self.speed
        self.dnamc.state.pd = self.height

    def setVehicleState(self, state):#done
        self.dnamc.state = state
    
    def setWindModel(self, windModel):#done
        self.wind = windModel

    def updateForces(self, state, controls, wind=None):#done half #PRIVATE
        res = Inputs.forcesMoments()
        if wind is not None:
            state.Va, state.alpha, state.beta = self.CalculateAirspeed(state, wind)
        else:
            state.Va = math.hypot(state.u, state.v, state.w)
            state.alpha = math.atan2(state.w, state.u)
            if math.isclose(state.Va, 0.0):
                state.beta = 0.0
            else:
                state.beta = math.asin(state.v/state.Va)

        grave = self.gravityForces(state)
        aero = self.aeroForces(state)
        con = self.controlForces(state, controls)
        
        res = grave + aero + con
        
        return res
    
    def __helper_va_a_b(self, state):#done #PRIVATE
        state.Va = math.hypot(state.u, state.v, state.w)
        state.alpha = math.atan2(state.w, state.u)
        if math.isclose(state.Va, 0.0):
            state.beta = 0.0
        else:
            state.beta = math.asin(state.v/state.Va)