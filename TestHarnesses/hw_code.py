from matplotlib import pyplot as plt
from ece163.Utilities import MatrixMath as mm
import numpy as np
import math
import random

tmax  =16
dt = 0.001
step = int(tmax/dt)
tau = 400
eta = 5e-4
t = [i*dt for i in range(step)]
nu = [0 for _ in range(step)]
for i in range(step-1):
    nu[i+1] = math.exp(-dt/tau)*nu[i]+random.gauss(0,eta)
plt.plot(t, nu)
plt.show()
