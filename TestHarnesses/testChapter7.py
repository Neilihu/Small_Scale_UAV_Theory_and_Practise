'''
Author: Neili Hu(nehu@ucsc.edu)
Date: February 2025
this test-file shows plots of related parameters
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

testVCLC = VCLC.VehicleClosedLoopControl(useSensors = True)
step = 500
t = [i*0.01 for i in range(step)]
gps_tn = [0 for _ in range(step)]
gps_te = [0 for _ in range(step)]
gps_td = [0 for _ in range(step)]
gps_nn = [0 for _ in range(step)]
gps_ne = [0 for _ in range(step)]
gps_nd = [0 for _ in range(step)]
gyr_tx = [0 for _ in range(step)]
gyr_ty = [0 for _ in range(step)]
gyr_tz = [0 for _ in range(step)]
gyr_nx = [0 for _ in range(step)]
gyr_ny = [0 for _ in range(step)]
gyr_nz = [0 for _ in range(step)]
acc_tx = [0 for _ in range(step)]
acc_ty = [0 for _ in range(step)]
acc_tz = [0 for _ in range(step)]
acc_nx = [0 for _ in range(step)]
acc_ny = [0 for _ in range(step)]
acc_nz = [0 for _ in range(step)]
mag_tx = [0 for _ in range(step)]
mag_ty = [0 for _ in range(step)]
mag_tz = [0 for _ in range(step)]
mag_nx = [0 for _ in range(step)]
mag_ny = [0 for _ in range(step)]
mag_nz = [0 for _ in range(step)]
baro_t = [0 for _ in range(step)]
baro_n = [0 for _ in range(step)]
pito_t = [0 for _ in range(step)]
pito_n = [0 for _ in range(step)]

for i in range(step):
    temp = testVCLC.update()
    gps_tn[i] = testVCLC.getSensorsModel().getSensorsTrue().gps_n
    gps_te[i] = testVCLC.getSensorsModel().getSensorsTrue().gps_e
    gps_td[i] = testVCLC.getSensorsModel().getSensorsTrue().gps_alt
    gps_nn[i] = testVCLC.getSensorsModel().getSensorsNoisy().gps_n
    gps_ne[i] = testVCLC.getSensorsModel().getSensorsNoisy().gps_e
    gps_nd[i] = testVCLC.getSensorsModel().getSensorsNoisy().gps_alt
    gyr_tx[i] = testVCLC.getSensorsModel().getSensorsTrue().gyro_x
    gyr_ty[i] = testVCLC.getSensorsModel().getSensorsTrue().gyro_y
    gyr_tz[i] = testVCLC.getSensorsModel().getSensorsTrue().gyro_z
    gyr_nx[i] = testVCLC.getSensorsModel().getSensorsNoisy().gyro_x
    gyr_ny[i] = testVCLC.getSensorsModel().getSensorsNoisy().gyro_y
    gyr_nz[i] = testVCLC.getSensorsModel().getSensorsNoisy().gyro_z
    acc_tx[i] = testVCLC.getSensorsModel().getSensorsTrue().accel_x
    acc_ty[i] = testVCLC.getSensorsModel().getSensorsTrue().accel_y
    acc_tz[i] = testVCLC.getSensorsModel().getSensorsTrue().accel_z
    acc_nx[i] = testVCLC.getSensorsModel().getSensorsNoisy().accel_x
    acc_ny[i] = testVCLC.getSensorsModel().getSensorsNoisy().accel_y
    acc_nz[i] = testVCLC.getSensorsModel().getSensorsNoisy().accel_z
    mag_tx[i] = testVCLC.getSensorsModel().getSensorsTrue().mag_x
    mag_ty[i] = testVCLC.getSensorsModel().getSensorsTrue().mag_y
    mag_tz[i] = testVCLC.getSensorsModel().getSensorsTrue().mag_z
    mag_nx[i] = testVCLC.getSensorsModel().getSensorsNoisy().mag_x
    mag_ny[i] = testVCLC.getSensorsModel().getSensorsNoisy().mag_y
    mag_nz[i] = testVCLC.getSensorsModel().getSensorsNoisy().mag_z
    baro_t[i] = testVCLC.getSensorsModel().getSensorsTrue().baro
    baro_n[i] = testVCLC.getSensorsModel().getSensorsNoisy().baro
    pito_t[i] = testVCLC.getSensorsModel().getSensorsTrue().pitot
    pito_n[i] = testVCLC.getSensorsModel().getSensorsNoisy().pitot

plt.close("all")

fig, a = plt.subplots(5, 3, figsize=(12, 8))
a = a.flatten()
a[0].plot(t, gps_tn, label = "n-true")
a[0].plot(t, gps_nn, label = "n-noisy")
a[0].legend()
a[0].set_title("n")

a[1].plot(t, gps_te, label = "e-true")
a[1].plot(t, gps_ne, label = "e-noisy")
a[1].legend()
a[1].set_title("e")

a[2].plot(t, gps_td, label = "alt-true")
a[2].plot(t, gps_nd, label = "alt-noisy")
a[2].legend()
a[2].set_title("height")

a[3].plot(t, gyr_tx, label = "x-true")
a[3].plot(t, gyr_nx, label = "x-noisy")
a[3].legend()
a[3].set_title("gyro-x")

a[4].plot(t, gyr_ty, label = "y-true")
a[4].plot(t, gyr_ny, label = "y-noisy")
a[4].legend()
a[4].set_title("gyro-y")

a[5].plot(t, gyr_tz, label = "z-true")
a[5].plot(t, gyr_nz, label = "z-noisy")
a[5].legend()
a[5].set_title("gyro-z")

a[6].plot(t, gyr_tx, label = "x-true")
a[6].plot(t, gyr_nx, label = "x-noisy")
a[6].legend()
a[6].set_title("acc-x")

a[7].plot(t, gyr_ty, label = "y-true")
a[7].plot(t, gyr_ny, label = "y-noisy")
a[7].legend()
a[7].set_title("acc-y")

a[8].plot(t, gyr_tz, label = "z-true")
a[8].plot(t, gyr_nz, label = "z-noisy")
a[8].legend()
a[8].set_title("acc-z")

a[9].plot(t, mag_tx, label = "x-true")
a[9].plot(t, mag_nx, label = "x-noisy")
a[9].legend()
a[9].set_title("mag-x")

a[10].plot(t, mag_ty, label = "y-true")
a[10].plot(t, mag_ny, label = "y-noisy")
a[10].legend()
a[10].set_title("mag-y")

a[11].plot(t, mag_tz, label = "z-true")
a[11].plot(t, mag_nz, label = "z-noisy")
a[11].legend()
a[11].set_title("mag-z")

a[12].plot(t, baro_t, label = "true")
a[12].plot(t, baro_n, label = "noisy")
a[12].legend()
a[12].set_title("baro")

a[13].plot(t, pito_t, label = "true")
a[13].plot(t, pito_n, label = "noisy")
a[13].legend()
a[13].set_title("pitot")

plt.tight_layout()
plt.show()