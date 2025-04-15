"""This file is a test harness for the module VehicleDynamicsModel. 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter3.py (from the root directory) -or-
python testChapter3.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehicleDynamicsModel module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-11], [0], [-1]]))
assert(not compareVectors([[1e8], [0], [-1]],[[1e8+1], [0], [-1]]))



failed = []
passed = []
def evaluateTest(test_name, boolean):
	"""evaluateTest prints the output of a test and adds it to one of two 
	global lists, passed and failed, which can be printed later"""
	if boolean:
		print(f"   passed {test_name}")
		passed.append(test_name)
	else:
		print(f"   failed {test_name}")
		failed.append(test_name)
	return boolean


#%% Derivative():
print("Beginning testing of VDM.Derivative(), subtest of [pe,pn,pd]")

cur_test = "Derivative test p_dot x dir"

testVDM = VDM.VehicleDynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
testState.pitch = 30*math.pi/180
testState.R = Rotations.euler2DCM(0.0,testState.pitch,0.0)
testState.u = 10
testDot = testVDM.derivative(testState, testFm)

print("With a velocity of u = 10 m/s, and pitch = 30deg:\n")
resultPdot = [[testDot.pn],[testDot.pe],[testDot.pd]]
expectedPdot = [[10*math.sqrt(3)/2],[0],[-10/2]]

if compareVectors(resultPdot,expectedPdot):
	print("passed!")
else:
	print("failed :(")



#%%  

"""
Students, add more tests here.  
You aren't required to use the testing framework we've started here, 
but it will work just fine.
"""
#-------------------------------------------------------------------
print("Begining Rexp() test all zero")
cur_test = "Rexp() test 1"
testVDM = VDM.VehicleDynamicsModel()
res =  testVDM.Rexp(testVDM.dt, testVDM.state, testVDM.dot)
exp = [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]]

if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
print("Begining Rexp() test state tiny value")
cur_test = "Rexp() test 2"
testVDM.state.p = 0.1
res =  testVDM.Rexp(testVDM.dt, testVDM.state, testVDM.dot)
exp = [[1.,0.,0.],[0., 0.9999995, 0.000999999],[0., -0.0009999998, 0.9999995]]

if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
print("Begining Rexp() test dot tiny value")
cur_test = "Rexp() test 3"
testVDM.state.p = 0
testVDM.dot.p = 0.1
res =  testVDM.Rexp(testVDM.dt, testVDM.state, testVDM.dot)
exp = [[1.,0.,0.],[0.0, 0.99999999, 0.00000499999],[0.0, -0.00000499999, 0.99999999]]

if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
print("Begining derivative() test ned all zero")
cur_test = "derivative() test 1.1"
testFm = Inputs.forcesMoments()
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.pn],[res.pe],[res.pd]]
exp = [[0.],[0.],[0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test ned tiny y-p-r")
cur_test = "derivative() test 1.2"
testVDM.state.yaw = 1
testVDM.state.pitch = 1
testVDM.state.roll = 1
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.pn],[res.pe],[res.pd]]
exp = [[0.],[0.],[0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test ned big y-p-r")
cur_test = "derivative() test 1.3"
testVDM.state.yaw = 100
testVDM.state.pitch = 100
testVDM.state.roll = 100
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.pn],[res.pe],[res.pd]]
exp = [[0.],[0.],[0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test ned tiny uvw")
cur_test = "derivative() test 1.4"
testVDM.state.yaw = 0
testVDM.state.pitch = 0
testVDM.state.roll = 0
testVDM.state.u = 0.1
testVDM.state.v = 0.1
testVDM.state.w = 0
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.pn],[res.pe],[res.pd]]
exp = [[0.1],[0.1],[0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test ned big uvw")
cur_test = "derivative() test 1.5"
testVDM.state.u = 100
testVDM.state.v = 0.1
testVDM.state.w = 2
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.pn],[res.pe],[res.pd]]
exp = [[100.],[0.1],[2.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test ned mixed input 1")
cur_test = "derivative() test 1.6"
testVDM.state.yaw = 12
testVDM.state.pitch = .1
testVDM.state.roll = 0
testVDM.state.u = 0.1
testVDM.state.v = 0.1
testVDM.state.w = 0
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.pn],[res.pe],[res.pd]]
exp = [[0.1376211121],[0.030996167034],[-0.009983341664]]  

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test ned mixed input 2")
cur_test = "derivative() test 1.7"
testVDM.state.yaw = 12
testVDM.state.pitch = .1
testVDM.state.roll = 100
testVDM.state.u = 0.1
testVDM.state.v = 100
testVDM.state.w = 2.77
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.pn],[res.pe],[res.pd]]
exp = [[43.041634686],[76.481888681],[-48.0168854383]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test y-p-r all zero")
cur_test = "derivative() test 2.1"
testVDM.state.yaw = 0
testVDM.state.pitch = 0
testVDM.state.roll = 0
testVDM.state.u = 0
testVDM.state.v = 0
testVDM.state.w = 0
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[0.],[0.],[0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test y-p-r tiny p-r")
cur_test = "derivative() test 2.2"
testVDM.state.pitch = 0.01
testVDM.state.roll = 0
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[0.],[0.],[0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test y-p-r big p-r")
cur_test = "derivative() test 2.3"
testVDM.state.pitch = 12
testVDM.state.roll = 0
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[0.],[0.],[0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test y-p-r tiny pqr")
cur_test = "derivative() test 2.4"
testVDM.state.pitch = 0
testVDM.state.roll = 0
testVDM.state.p = 0.1
testVDM.state.q = 3
testVDM.state.r = 0.05
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[0.05],[3.],[0.1]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test y-p-r big pqr")
cur_test = "derivative() test 2.5"
testVDM.state.pitch = 0
testVDM.state.roll = 0
testVDM.state.p = 12
testVDM.state.q = 200
testVDM.state.r = 135
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[135.],[200.],[12.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test y-p-r mixed input 1")
cur_test = "derivative() test 2.6"
testVDM.state.pitch = 1
testVDM.state.roll = 0.4
testVDM.state.p = 1
testVDM.state.q = 20
testVDM.state.r = 15
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[39.985544243],[12.579944745],[34.64667529]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test y-p-r mixed input 2")
cur_test = "derivative() test 2.7"
testVDM.state.pitch = 10
testVDM.state.roll = 456
testVDM.state.p = 122
testVDM.state.q = 201
testVDM.state.r = 153
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[270.93905609],[-110.1261805],[-25.396566281]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test uvw all zero")
cur_test = "derivative() test 3.1"
testVDM.state.pitch = 0
testVDM.state.roll = 0
testVDM.state.p = 0
testVDM.state.q = 0
testVDM.state.r = 0

res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.u],[res.v],[res.w]]
exp = [[0.],[0.],[0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test uvw tiny pqr")
cur_test = "derivative() test 3.2"
testVDM.state.p = 0
testVDM.state.q = 0.3
testVDM.state.r = 0
testVDM.state.u = 0
testVDM.state.v = 0
testVDM.state.w = 0
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.u],[res.v],[res.w]]
exp = [[0.],[0.],[0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test uvw big pqr")
cur_test = "derivative() test 3.3"
testVDM.state.p = 100
testVDM.state.q = 200
testVDM.state.r = 300
testVDM.state.u = 0
testVDM.state.v = 0
testVDM.state.w = 0
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.u],[res.v],[res.w]]
exp = [[0.],[0.],[0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test uvw tiny uvw")
cur_test = "derivative() test 3.4"
testVDM.state.p = 0
testVDM.state.q = 0
testVDM.state.r = 0
testVDM.state.u = 0.1
testVDM.state.v = 0.2
testVDM.state.w = 0.09
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.u],[res.v],[res.w]]
exp = [[0.],[0.],[0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test uvw big uvw")
cur_test = "derivative() test 3.5"
testVDM.state.p = 0
testVDM.state.q = 0
testVDM.state.r = 0
testVDM.state.u = 123
testVDM.state.v = 456
testVDM.state.w = 789
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.u],[res.v],[res.w]]
exp = [[0.],[0.],[0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test uvw mixed input 1")
cur_test = "derivative() test 3.6"
testVDM.state.p = 0.4
testVDM.state.q = 0.6
testVDM.state.r = 0.09
testVDM.state.u = 1
testVDM.state.v = 4
testVDM.state.w = 0.88
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.u],[res.v],[res.w]]
exp = [[-0.168],[0.262],[-1.0]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test uvw mixed input 2")
cur_test = "derivative() test 3.7"
testVDM.state.p = 42
testVDM.state.q = 64
testVDM.state.r = 90
testVDM.state.u = 177
testVDM.state.v = 4
testVDM.state.w = 88
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.u],[res.v],[res.w]]
exp = [[-5272.0], [-12234.0], [11160.0]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test pqr all zero")
cur_test = "derivative() test 4.1"
testVDM.state.p = 0
testVDM.state.q = 0
testVDM.state.r = 0
testVDM.state.u = 0
testVDM.state.v = 0
testVDM.state.w = 0
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.p],[res.q],[res.r]]
exp = [[0.0], [0.0], [0.0]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test pqr tiny pqr")
cur_test = "derivative() test 4.2"
testVDM.state.p = 0.1
testVDM.state.q = 0.2
testVDM.state.r = 0.3
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.p],[res.q],[res.r]]
exp = [[-0.04404983969], [0.03318942731], [-0.010653553553]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining derivative() test pqr big pqr")
cur_test = "derivative() test 4.3"
testVDM.state.p = 12
testVDM.state.q = 57
testVDM.state.r = 112
res = testVDM.derivative(testVDM.state, testFm)
resb = [[res.p],[res.q],[res.r]]
exp = [[-4862.307817], [2422.0814096], [-890.5661519]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test ned,uvw,pqr all zero")
cur_test = "IntegrateState() test 1.1"
testVDM.state.p = 0
testVDM.state.q = 0
testVDM.state.r = 0
testVDM.dot.p = 0
testVDM.dot.q = 0
testVDM.dot.r = 0

testVDM.state.u = 0
testVDM.state.v = 0
testVDM.state.w = 0
testVDM.dot.u = 0
testVDM.dot.v = 0
testVDM.dot.w = 0

testVDM.state.pn = 0
testVDM.state.pe = 0
testVDM.state.pd = 0
testVDM.dot.pn = 0
testVDM.dot.pe = 0
testVDM.dot.pd = 0
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.p,res.q,res.r],[res.u,res.v,res.w],[res.pn,res.pe,res.pd]]
exp = [[0.,0.,0.], [0.,0.,0.], [0.,0.,0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test ned,uvw,pqr tiny state")
cur_test = "IntegrateState() test 1.2"
testVDM.state.p = 0.1
testVDM.state.q = 0.1
testVDM.state.r = 0.1
testVDM.dot.p = 0
testVDM.dot.q = 0
testVDM.dot.r = 0

testVDM.state.u = 0.1
testVDM.state.v = 0.1
testVDM.state.w = 0.1
testVDM.dot.u = 0
testVDM.dot.v = 0
testVDM.dot.w = 0

testVDM.state.pn = 0.1
testVDM.state.pe = 0.1
testVDM.state.pd = 0.1
testVDM.dot.pn = 0
testVDM.dot.pe = 0
testVDM.dot.pd = 0
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.p,res.q,res.r],[res.u,res.v,res.w],[res.pn,res.pe,res.pd]]
exp = [[0.1,0.1,0.1], [0.1,0.1,0.1], [0.1,0.1,0.1]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test ned,uvw,pqr big state")
cur_test = "IntegrateState() test 1.3"
testVDM.state.p = 100
testVDM.state.q = 100
testVDM.state.r = 100
testVDM.dot.p = 0
testVDM.dot.q = 0
testVDM.dot.r = 0

testVDM.state.u = 100
testVDM.state.v = 100
testVDM.state.w = 100
testVDM.dot.u = 0
testVDM.dot.v = 0
testVDM.dot.w = 0

testVDM.state.pn = 100
testVDM.state.pe = 100
testVDM.state.pd = 100
testVDM.dot.pn = 0
testVDM.dot.pe = 0
testVDM.dot.pd = 0
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.p,res.q,res.r],[res.u,res.v,res.w],[res.pn,res.pe,res.pd]]
exp = [[100,100,100], [100,100,100], [100,100,100]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test ned,uvw,pqr tiny dot")
cur_test = "IntegrateState() test 1.4"
testVDM.state.p = 0
testVDM.state.q = 0
testVDM.state.r = 0
testVDM.dot.p = 0.1
testVDM.dot.q = 0.1
testVDM.dot.r = 0.1

testVDM.state.u = 0
testVDM.state.v = 0
testVDM.state.w = 0
testVDM.dot.u = 0.1
testVDM.dot.v = 0.1
testVDM.dot.w = 0.1

testVDM.state.pn = 0
testVDM.state.pe = 0
testVDM.state.pd = 0
testVDM.dot.pn = 0.1
testVDM.dot.pe = 0.1
testVDM.dot.pd = 0.1
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.p,res.q,res.r],[res.u,res.v,res.w],[res.pn,res.pe,res.pd]]
exp = [[0.001,0.001,0.001], [0.001,0.001,0.001], [0.001,0.001,0.001]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test ned,uvw,pqr big dot")
cur_test = "IntegrateState() test 1.5"
testVDM.state.p = 0
testVDM.state.q = 0
testVDM.state.r = 0
testVDM.dot.p = 100
testVDM.dot.q = 100
testVDM.dot.r = 100

testVDM.state.u = 0
testVDM.state.v = 0
testVDM.state.w = 0
testVDM.dot.u = 100
testVDM.dot.v = 100
testVDM.dot.w = 100

testVDM.state.pn = 0
testVDM.state.pe = 0
testVDM.state.pd = 0
testVDM.dot.pn = 100
testVDM.dot.pe = 100
testVDM.dot.pd = 100
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.p,res.q,res.r],[res.u,res.v,res.w],[res.pn,res.pe,res.pd]]
exp = [[1,1,1], [1,1,1], [1,1,1]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test ned,uvw,pqr mixed inputs 1")
cur_test = "IntegrateState() test 1.6"
testVDM.state.p = 1
testVDM.state.q = 1
testVDM.state.r = 1
testVDM.dot.p = 10
testVDM.dot.q = 10
testVDM.dot.r = 10

testVDM.state.u = 1
testVDM.state.v = 1
testVDM.state.w = 1
testVDM.dot.u = 10
testVDM.dot.v = 10
testVDM.dot.w = 10

testVDM.state.pn = 1
testVDM.state.pe = 1
testVDM.state.pd = 1
testVDM.dot.pn = 10
testVDM.dot.pe = 10
testVDM.dot.pd = 10
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.p,res.q,res.r],[res.u,res.v,res.w],[res.pn,res.pe,res.pd]]
exp = [[1.1,1.1,1.1], [1.1,1.1,1.1], [1.1,1.1,1.1]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test ned,uvw,pqr mixed inputs 2")
cur_test = "IntegrateState() test 1.7"
testVDM.state.p = 100
testVDM.state.q = 100
testVDM.state.r = 100
testVDM.dot.p = 1000
testVDM.dot.q = 1000
testVDM.dot.r = 1000

testVDM.state.u = 100
testVDM.state.v = 100
testVDM.state.w = 100
testVDM.dot.u = 1000
testVDM.dot.v = 1000
testVDM.dot.w = 1000

testVDM.state.pn = 100
testVDM.state.pe = 100
testVDM.state.pd = 100
testVDM.dot.pn = 1000
testVDM.dot.pe = 1000
testVDM.dot.pd = 1000
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.p,res.q,res.r],[res.u,res.v,res.w],[res.pn,res.pe,res.pd]]
exp = [[110,110,110], [110,110,110], [110,110,110]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test y-p-r all zero")
cur_test = "IntegrateState() test 2.1"
testVDM.state.p = 0
testVDM.state.q = 0
testVDM.state.r = 0
testVDM.dot.p = 0
testVDM.dot.q = 0
testVDM.dot.r = 0

testVDM.state.u = 0
testVDM.state.v = 0
testVDM.state.w = 0
testVDM.dot.u = 0
testVDM.dot.v = 0
testVDM.dot.w = 0

testVDM.state.pn = 0
testVDM.state.pe = 0
testVDM.state.pd = 0
testVDM.dot.pn = 0
testVDM.dot.pe = 0
testVDM.dot.pd = 0
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[0.], [0.], [0.]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test y-p-r tiny state")
cur_test = "IntegrateState() test 2.2"
testVDM.state.p = 0.1
testVDM.state.q = 0.2
testVDM.state.r = 0.3
testVDM.dot.p = 0
testVDM.dot.q = 0
testVDM.dot.r = 0
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[0.0030010034963], [0.0019984966654], [0.001002999834]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test y-p-r big state")
cur_test = "IntegrateState() test 2.3"
testVDM.state.p = 15
testVDM.state.q = 65
testVDM.state.r = 981
testVDM.dot.p = 0
testVDM.dot.q = 0
testVDM.dot.r = 0
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[-0.9366038956], [-1.5707963267], [-1.4995677690]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test y-p-r tiny dot")
cur_test = "IntegrateState() test 2.4"
testVDM.state.p = 0
testVDM.state.q = 0
testVDM.state.r = 0
testVDM.dot.p = 0.2
testVDM.dot.q = 0.09
testVDM.dot.r = 0.122
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[0.0000061000224], [0.0000044999694], [0.0000100000137]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test y-p-r big dot")
cur_test = "IntegrateState() test 2.5"
testVDM.state.p = 0
testVDM.state.q = 0
testVDM.state.r = 0
testVDM.dot.p = 2
testVDM.dot.q = 92
testVDM.dot.r = 122
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[0.006100273012], [0.004599666462], [0.00011403016525]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test y-p-r mixed inputs 1")
cur_test = "IntegrateState() test 2.6"
testVDM.state.p = 0.9
testVDM.state.q = 2.77
testVDM.state.r = 12.98
testVDM.dot.p = 0.9
testVDM.dot.q = 8.56
testVDM.dot.r = 43.3
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[0.13212386109], [0.027449889716], [0.01087489655]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#-------------------------------------------------------------------
print("Begining IntegrateState() test y-p-r mixed inputs 2")
cur_test = "IntegrateState() test 2.7"
testVDM.state.p = 90
testVDM.state.q = 277
testVDM.state.r = 1298
testVDM.dot.p = 90
testVDM.dot.q = 856
testVDM.dot.r = 433
res = testVDM.IntegrateState(testVDM.dt, testVDM.state, testVDM.dot)
resb = [[res.yaw],[res.pitch],[res.roll]]
exp = [[-0.7707662978], [-1.5707963267], [-1.3920609645]]

if not evaluateTest(cur_test, compareVectors(exp, resb)):
	print(f"{exp} != {resb}")
#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]