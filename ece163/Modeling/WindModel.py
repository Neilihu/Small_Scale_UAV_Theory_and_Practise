'''
Author: Neili Hu (nehu@ucsc.edu)
Date: January 2025
This file implements wind state that used for aerodynamic model
'''
import math
import random
from ..Containers import States
from ..Utilities import MatrixMath
from ..Constants import VehiclePhysicalConstants as VPC

class WindModel:
    def __init__(self, dT=VPC.dT, Va=VPC.InitialSpeed, drydenParameters=VPC.DrydenNoWind): #done
        self.dt = dT
        self.va = Va

        self.ph_u = .0
        self.gama_u = .0
        self.h_u = .0

        self.ph_v = [[.0, .0],[.0, .0]]
        self.gama_v = [[.0], [.0]]
        self.h_v = [[.0, .0]]

        self.ph_w = [[.0, .0],[.0, .0]]        
        self.gama_w = [[.0], [.0]]
        self.h_w = [[.0, .0]]

        self.CreateDrydenTransferFns(dT, Va, drydenParameters)
        self.wind = States.windState()

        self.xu = .0
        self.xv = [[.0],[.0]]
        self.xw = [[.0],[.0]]
        
    def CreateDrydenTransferFns(self, dT, Va, drydenParameters):#done #PRIVATE
        if Va <= 0:
            raise AttributeError('Va <= 0')
        
        if drydenParameters == VPC.DrydenNoWind:
            self.ph_u = 1.
            self.gama_u = .0
            self.h_u = 1.

            self.ph_v = [[1., .0],[.0, 1.]]
            self.gama_v = [[.0], [.0]]
            self.h_v = [[1., 1.]]

            self.ph_w = [[1., .0],[.0, 1.]]        
            self.gama_w = [[.0], [.0]]
            self.h_w = [[1., 1.]]

        else:            
            temp = drydenParameters
            du = Va/temp.Lu
            dv = Va/temp.Lv
            dw = Va/temp.Lw
            bu = math.exp(-1*dT*du)
            bv = math.exp(-1*dT*dv)
            bw = math.exp(-1*dT*dw)

            self.ph_u = bu
            self.gama_u = temp.Lu/Va*(1-bu)
            self.h_u = temp.sigmau*math.sqrt(2*du/math.pi)

            self.ph_v = [[bv*(1-dv*dT), bv*(-1*dv**2*dT)],[bv*dT, bv*(1+dv*dT)]]
            self.gama_v = [[bv*dT], [bv*(((temp.Lv/Va)**2)*(math.exp(dT*dv)-1)-temp.Lv/Va*dT)]]
            self.h_v = [[temp.sigmav*math.sqrt(3*dv/math.pi), temp.sigmav*math.sqrt(Va**3/math.pi/temp.Lv**3)]]

            self.ph_w = [[bw*(1-dw*dT), bw*(-1*dw**2*dT)],[bw*dT, bw*(1+dw*dT)]]
            self.gama_w = [[bw*dT], [bw*(((temp.Lw/Va)**2)*(math.exp(dT*dw)-1)-temp.Lw/Va*dT)]]
            self.h_w = [[temp.sigmaw*math.sqrt(3*dw/math.pi), temp.sigmaw*math.sqrt(Va**3/math.pi/temp.Lw**3)]]
    
    def Update(self, uu=None, uv=None, uw=None):#done
        uu = random.gauss(0,1) if uu is None else uu
        uv = random.gauss(0,1) if uv is None else uv
        uw = random.gauss(0,1) if uw is None else uw
        self.xu = self.ph_u*self.xu + self.gama_u*uu
        self.wind.Wu = self.h_u*(self.xu)

        temp1 = MatrixMath.scalarMultiply(uv, self.gama_v)
        temp2 = MatrixMath.multiply(self.ph_v, self.xv)
        self.xv = MatrixMath.add(temp1,temp2)
        self.wind.Wv = MatrixMath.multiply(self.h_v, self.xv)[0][0]

        temp1 = MatrixMath.scalarMultiply(uw, self.gama_w)
        temp2 = MatrixMath.multiply(self.ph_w, self.xw)
        self.xw = MatrixMath.add(temp1,temp2)
        self.wind.Ww = MatrixMath.multiply(self.h_w, self.xw)[0][0]

    def getDrydenTransferFns(self):#done
        Phi_u = [[self.ph_u]]
        Gamma_u = [[self.gama_u]]
        H_u = [[self.h_u]]

        Phi_v = self.ph_v
        Gamma_v = self.gama_v
        H_v = self.h_v

        Phi_w = self.ph_w
        Gamma_w = self.gama_w
        H_w = self.h_w

        return Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w
    
    def getWind(self):#done
        return self.wind
    
    def reset(self):#done
        self.wind = States.windState()
        self.CreateDrydenTransferFns(self.dt, self.va, drydenParameters = VPC.DrydenNoWind)
        self.va = VPC.InitialSpeed
        self.xu = .0
        self.xv = [[.0],[.0]]
        self.xw = [[.0],[.0]]
    
    def setWind(self, windState):#done
        self.wind = windState

    def setWindModelParameters(self, Wn=.0, We=.0, Wd=.0, drydenParameters=VPC.DrydenNoWind):#done
        self.wind.Wn = Wn
        self.wind.We = We
        self.wind.Wd = Wd
        self.CreateDrydenTransferFns(self.dt, self.va, drydenParameters)