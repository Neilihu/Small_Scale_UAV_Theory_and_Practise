'''
Auther: Neili Hu (nehu@ucsc.edu)
Data:Janurary 2025
This file implmemnts some dynatic formulas for the aircarft.
'''
import math
from ..Containers import States
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

class VehicleDynamicsModel:
    def __init__(self):
        self.dt = VPC.dT
        self.state = States.vehicleState()
        self.dot = States.vehicleState()

    def ForwardEuler(self, dt, state, dot):#PRIVATE
        res = States.vehicleState()
        res.pn = state.pn + dot.pn*dt
        res.pe = state.pe + dot.pe*dt
        res.pd = state.pd + dot.pd*dt
        res.u = state.u + dot.u*dt
        res.v = state.v + dot.v*dt
        res.w = state.w + dot.w*dt
        res.p = state.p + dot.p*dt
        res.q = state.q + dot.q*dt
        res.r = state.r + dot.r*dt

        return res
    
    def IntegrateState(self, dt, state, dot):#PRIVATE
    #forward euler-------------------------------------------------
        res = self.ForwardEuler(dt, state, dot)
    #--------------------------------------------------------------

    #attitude update-----------------------------------------------
        res.R = MatrixMath.multiply(self.Rexp(dt, state, dot), state.R)
        [res.yaw, res.pitch, res.roll] = Rotations.dcm2Euler(res.R)
    #--------------------------------------------------------------

    #extra update--------------------------------------------------
        res.chi = math.atan2(dot.pe, dot.pn)
        res.Va = state.Va
        res.alpha = state.alpha
        res.beta = state.beta
    #--------------------------------------------------------------

        return res
    
    def Rexp(self, dt, state, dot):#PRIVATE
        #Rotation Rate Considerations in attitude.pdf--------------
        p=state.p+dot.p*dt/2
        q=state.q+dot.q*dt/2
        r=state.r+dot.r*dt/2
        #----------------------------------------------------------
        w = MatrixMath.skew(p, q, r)

        w_norm = p*p + q*q + r*r
        w2 = MatrixMath.multiply(w,w)

        i = [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]
        dt2 =dt**2
        
        #add extra two items and make the function working
        #I think there should be a better way to do so-------------
        s = MatrixMath.scalarMultiply((120*dt-20*(dt**3)*w_norm+(dt**5)*(w_norm**2))/120
                                      -(dt**7)*(w_norm**3)/5040
                                      +(dt**9)*(w_norm**4)/362880
                                      -(dt**11)*(w_norm**5)/39916800, w)
        c = MatrixMath.scalarMultiply((360*dt2-30*(dt2**2)*w_norm+(dt2**3)*(w_norm**2))/720
                                      -(dt2**4)*(w_norm**3)/40320
                                      +(dt2**5)*(w_norm**4)/3628800
                                      -(dt2**6)*(w_norm**5)/479001600, w2)
        #----------------------------------------------------------

        e = MatrixMath.subtract(c,s)
        e = MatrixMath.add(i,e)

        return e
    
    def Update(self, forcesMoments):
        self.dot = self.derivative(self.state, forcesMoments)
        self.state = self.IntegrateState(self.dt, self.state, self.dot)
    
    def derivative(self, state, forcesMoments):
        dot = States.vehicleState()

        #pned-----------------------------------------------------------------------------------
        temp = Rotations.euler2DCM(state.yaw, state.pitch, state.roll)
        [[dot.pn], [dot.pe], [dot.pd]] = MatrixMath.multiply(MatrixMath.transpose(temp), [[state.u],[state.v],[state.w]])
        #---------------------------------------------------------------------------------------

        #yaw,pitch,roll-------------------------------------------------------------------------
        p = state.pitch
        r = state.roll
        temp=[[1, math.sin(r)*math.tan(p), math.cos(r)*math.tan(p)],
              [0, math.cos(r), -1*math.sin(r)],
              [0, math.sin(r)/math.cos(p), math.cos(r)/math.cos(p)]]
        [[dot.roll], [dot.pitch], [dot.yaw]] = MatrixMath.multiply(temp, [[state.p],[state.q],[state.r]])
        #---------------------------------------------------------------------------------------

        #uvw------------------------------------------------------------------------------------
        mass = VPC.mass
        dot.u = state.r*state.v - state.q*state.w + forcesMoments.Fx/mass
        dot.v = state.p*state.w - state.r*state.u + forcesMoments.Fy/mass
        dot.w = state.q*state.u - state.p*state.v + forcesMoments.Fz/mass
        #---------------------------------------------------------------------------------------

        #pqr------------------------------------------------------------------------------------
        temp = MatrixMath.multiply(VPC.Jbody, [[state.p],[state.q],[state.r]])
        temp = MatrixMath.multiply(MatrixMath.skew(-1*state.p, -1*state.q, -1*state.r), temp)
        temp = MatrixMath.add(temp, [[forcesMoments.Mx],[forcesMoments.My],[forcesMoments.Mz]])
        [[dot.p], [dot.q], [dot.r]] = MatrixMath.multiply(VPC.JinvBody, temp)
        #---------------------------------------------------------------------------------------

        #annoying R-----------------------------------------------------------------------------
        dot.R = MatrixMath.multiply(MatrixMath.skew(-1*state.p, -1*state.q, -1*state.r), state.R)
        #---------------------------------------------------------------------------------------

        return dot

    def setVehicleState(self, state):
        self.state = state

    def getVehicleState(self):
        return self.state
    
    def setVehicleDerivative(self, dot):
        self.dot = dot

    def getVehicleDerivative(self):
        return self.dot
    
    def reset(self):
        self.state = States.vehicleState()
        self.dot = States.vehicleState()

    




    