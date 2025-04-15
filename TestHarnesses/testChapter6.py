'''
Author: Neili Hu(nehu@ucsc.edu)
Date: February 2025
this test file shows plots of related parameters
'''
import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Modeling.WindModel as WM
import ece163.Controls.VehicleTrim as VehicleTrim
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Controls.VehicleClosedLoopControl as VCLC
import ece163.Containers.Controls as Controls
import matplotlib.pyplot as plt

testVCLC = VCLC.VehicleClosedLoopControl()
step = 5000
t = [i*0.01 for i in range(step)]
ailron = [0 for _ in range(step)]
rudder = [0 for _ in range(step)]
pitch = [0 for _ in range(step)]
roll = [0 for _ in range(step)]
chi = [0 for _ in range(step)]

for i in range(step):
    temp = testVCLC.Update()
    pitch[i] = testVCLC.getVehicleState().pitch
    roll[i] = testVCLC.getVehicleState().roll
    chi[i] = testVCLC.getVehicleState().chi
plt.close("all")

plt.plot(t, pitch, label = "pitch")
plt.plot(t, roll, label = "roll")
plt.plot(t, chi, label = "Chi")
plt.xlabel("Time in sec")
plt.ylabel("Magnitude")
plt.title("Coordiante turn wihtout wind")
plt.legend()
plt.show()
