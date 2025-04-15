from matplotlib import pyplot as plt
from ece163.Utilities import MatrixMath as mm
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Modeling import VehicleDynamicsModel as VDM
from ece163.Containers import States
from ece163 import Pthfollowing
from ece163 import Containers
import numpy as np
import math
import random

print('Welcome to the pathfollowing demo script')
print('After typing and you get no response, close & open the bash and run command winpty python hw_code.py')
print('Please following the guidance to generate result.')
temp = input('type \'default\' for default setting or set self setting\nYour input:')
if temp == 'default':
	line = Containers.Controls.intendedPath(mode = 1, oX = 150, oY = 150, oZ = 10, R = 90, Direction = 1)
	time = 3000
else:
	line = Containers.Controls.intendedPath(mode = 1, oZ = 10)
	while 1:
		print('\nChoose your path mode: \'straight\' or \'circular\'')
		mode = input('Your input:')
		if mode == 'straight':
			print('not implemented yet!')
		elif mode == 'circular':
			break
		elif mode == 'exit':
			exit()
		else:
			print('un-recognized input!')
	while 1:
		print('\nSet the center of the circle in range [100, 200] for x, y position')
		print ('Example [100,111]')
		path1 = input('Your input:')
		if path1 == 'exit':
			exit()
		elif path1[0] != '[' or path1[4] !=',' or path1[-1] != ']':
			print('un-recognized input!')
		else:
			x = int(path1[1:3])
			y = int(path1[5:7])
			if 100 < x or x > 200:
				print('x out of range')
			elif 100 < y or y > 200:
				print('y out of range')
			else:
				line.oX = x
				line.oY = y
				break
	while 1:
		print('\nset radius in range [50, 99]')
		print('Example 88')
		radius1 = input('Your input:')
		if radius1 == 'exit':
			exit()
		elif int(radius1) > 99 or int(radius1) < 50:
			print('invelid input!')
		else:
			line.R = int(radius1)
			break
	
	while 1:
		print('\nset simulation time in range [10, 100] sconds')
		print('Example: 21')
		time1 = input('Your input:')
		if time1 == 'exit':
			exit()
		elif int(time1) > 100 or int(time1) < 10:
			print('invalid input')
		else:
			time = int(time1)*100
			break
	while 1:
		print('\nset direction wither 1 or -1')
		dirc = input('Your input:')
		if dirc == 'exit':
			exit()
		elif int(dirc) == 1:
			line.Direction = 1
			break
		elif int(dirc) == -1:
			line.Direction = -1
			break
		else:
			print('invalid input') 

print('--------------------------------------------------')
print('Generating plot...')
test_path = Pthfollowing.Pathfollow(VPC.dT)
t = [0.01*i for i in range(time)]
pn = [0 for _ in range(time)]
pe = [0 for _ in range(time)]
pitch = [0 for _ in range(time)]
for i in range(time):
    test_path.update(line)
    pn[i] = test_path.getstate().pn
    pe[i] = test_path.getstate().pe 
    pitch[i] = test_path.getstate().pitch

plt.close("all")
fig, ax = plt.subplots()
ax.add_patch(plt.Circle((line.oX,line.oY), line.R, fill = False))
plt.plot(pe, pn, label = 'trajectory')
ax.set_xlim(-150, 400)
ax.set_ylim(-150, 400)
ax.set_aspect('equal')
ax.plot(0,0,marker="o", markersize=8, color = 'red', label = 'start position')
ax.plot(pe[time-1],pn[time-1],marker="o", markersize=8, color = 'green', label = 'end posiiton')
plt.title(f"{time/100} seconds simulation trajectory plot")
plt.legend()
plt.xlabel('pe')
plt.ylabel('pn')
plt.show()
plt.plot(t, pitch)
plt.xlabel('time')
plt.ylabel('pitch angle')
plt.title('pitch angle plot')
plt.show()