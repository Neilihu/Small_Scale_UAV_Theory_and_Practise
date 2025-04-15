'''
Author: Neili Hu (nehu@ucsc.edu)
Date: February 2025
This is a test file for VehicleAerodynamicsModel completely
'''
import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleAerodynamicsModel as md
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Constants.VehiclePhysicalConstants as VPC
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
'''
personal test place
'''
print("test gravityForces()")
cur_test = "gravity test"
testmodel = md.VehicleAerodynamicsModel()
testState = States.vehicleState()
res = testmodel.gravityForces(testState)
res = [[res.Fx],[res.Fy],[res.Fz]]
exp = [[0.],[0.],[107.91]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
print("test CalculateCoeff_alpha()")
cur_test = "coeff alpha = 0"
[res[0][0],res[1][0],res[2][0]] = testmodel.CalculateCoeff_alpha(0)
exp = [[0.2299999999730887], [0.06122729464970145], [0.0135]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
cur_test = "coeff alpha = 1"
[res[0][0],res[1][0],res[2][0]] = testmodel.CalculateCoeff_alpha(1)
exp = [[0.9092974268419375], [1.41614683654528], [-2.7265]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
cur_test = "coeff alpha = -1"
[res[0][0],res[1][0],res[2][0]] = testmodel.CalculateCoeff_alpha(-1)
exp = [[-0.9092974268404209], [1.4161468365448853], [2.75350000000000]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
print("test aeroForces()")
cur_test = "aeroForces 1"
res = testmodel.aeroForces(testState)
res = [[res.Fx],[res.Fy],[res.Fz],[res.Mx],[res.My],[res.Mz]]
exp = [[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
cur_test = "aeroForces 2"
testState.Va = 1
testState.beta = 1
testState.alpha = 10
testState.q = 3
res1 = testmodel.aeroForces(testState)
res = [[res1.Fx],[res1.Fy],[res1.Fz]]
exp = [[-0.42974518261427047], [-0.3417799], [1.0422780966986651]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
res = [[res1.Mx],[res1.My],[res1.Mz]]
exp = [[-0.13128114714000003], [-2.5352942588826544], [0.07371941339400001]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
print("test controlForces()")
cur_test = "controlForces 1"
testState = States.vehicleState()
testControl = Inputs.controlInputs()
res1 = testmodel.controlForces(testState, testControl)
res = [[res1.Fx],[res1.Fy],[res1.Fz]]
exp = [[21.817680754436033], [0.0], [0.0]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
res = [[res1.Mx],[res1.My],[res1.Mz]]
exp = [[-0.6194943564776727], [0.0], [0.0]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
cur_test = "controlForces 2"
testControl.Elevator = 1
testState.Va = 1
res1 = testmodel.controlForces(testState, testControl)
res = [[res1.Fx],[res1.Fy],[res1.Fz]]
exp = [[21.22616534288717], [0.0], [-0.04533815000000001]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
res = [[res1.Mx],[res1.My],[res1.Mz]]
exp = [[-0.6380953261943871], [-0.065580099453],[0.0]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
print("test CalculatePropForces()")
cur_test = "CalculatePropForces 1"
res = testmodel.CalculatePropForces(0,0)
res = [[res[0]],[res[1]],[0]]
exp = [[0.00018320594739300043], [-0.0000052019759460],[0]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
cur_test = "CalculatePropForces 2"
res = testmodel.CalculatePropForces(0,1)
res = [[res[0]],[res[1]],[0]]
exp = [[84.56952909917005], [-2.401279338375964], [0]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
cur_test = "CalculatePropForces 3"
res = testmodel.CalculatePropForces(1,1)
res = [[res[0]],[res[1]],[0]]
exp = [[83.39978539318385], [-2.4391985428760563], [0]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
print("test updateForces()")
cur_test = "updateForces 1"
testState = States.vehicleState()
testControl = Inputs.controlInputs()
res1 = testmodel.updateForces(testState, testControl)
res = [[res1.Fx],[res1.Fy],[res1.Fz]]
exp = [[21.817680754436033], [0.0], [107.91000000000001]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
res = [[res1.Mx],[res1.My],[res1.Mz]]
exp = [[-0.6194943564776727], [0.0], [0.0]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
cur_test = "updateForces 2"
testControl.Aileron = 1
testState.Va = 20
res1 = testmodel.updateForces(testState, testControl)
res = [[res1.Fx],[res1.Fy],[res1.Fz]]
exp = [[-10.785844108480342], [10.46265], [75.82454000375418]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
res = [[res1.Mx],[res1.My],[res1.Mz]]
exp = [[68.68468093663846], [0.35770963338], [-4.4433619032]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
cur_test = "updateForces wind"
testControl = Inputs.controlInputs()
testWind = States.windState()
res1 = testmodel.updateForces(testState, testControl, testWind)
res = [[res1.Fx],[res1.Fy],[res1.Fz]]

exp = [[21.817680754436033], [0.0], [107.91000000000001]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
res = [[res1.Mx],[res1.My],[res1.Mz]]
exp = [[-0.6194943564776727], [0.0], [0.0]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
print("test CalculateAirspeed()")
cur_test = "CalculateAirspeed 1"
testState = States.vehicleState()
testWind = States.windState()
res = testmodel.CalculateAirspeed(testState, testWind)
res = [[res[0]],[res[1]],[res[2]]]
exp = [[0.0], [0.0], [0.0]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
cur_test = "CalculateAirspeed 2"
testState.yaw = 10
testState.pitch = 15
testWind.We = 3
res = testmodel.CalculateAirspeed(testState, testWind)
res = [[res[0]],[res[1]],[res[2]]]
exp = [[3.0], [2.433629385640827], [0.9955742875642764]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
cur_test = "CalculateAirspeed 3"
testState.yaw = 0.01
testState.roll = 17
testWind.Wn = 20
res = testmodel.CalculateAirspeed(testState, testWind)
res = [[res[0]],[res[1]],[res[2]]]
exp = [[20.223748416156685], [0.05856445314048174], [0.7171766199908964]]
if not evaluateTest(cur_test, compareVectors(exp, res)):
	print(f"{exp} != {res}")
#-------------------------------------------------------------------
print("test Update() 10000x2 times")
cur_test = "update "
testControl = Inputs.controlInputs()
testmodel.wind.CreateDrydenTransferFns(0.01, VPC.InitialSpeed, VPC.DrydenLowAltitudeLight)
for i in range(10000):
	testmodel.wind.Update()
	testmodel.Update(testControl)
res1 = testmodel.dnamc.state
testmodel.reset()
testmodel.wind.CreateDrydenTransferFns(0.01, VPC.InitialSpeed, VPC.DrydenLowAltitudeLight)
for i in range(10000):
	testmodel.wind.Update()
	testmodel.Update(testControl)
res2 = testmodel.dnamc.state
res = [res1.pn/res2.pn, res1.pe/res2.pe, res1.pd/res2.pd,
	   res1.u/res2.u, res1.v/res2.v, res1.w/res2.w,
	   res1.yaw/res2.yaw, res1.pitch/res2.pitch, res1.roll/res2.roll,
	   res1.p/res2.p, res1.q/res2.q, res1.r/res2.r,
	   res1.Va/res2.Va, res1.alpha/res2.alpha, res1.beta/res2.beta, res1.chi/res2.chi]
res = math.fsum(res)/16*100
print('Difference between two run is '+'%.3f'%res+'%')
print('The difference concentrated in +25%-10%')
#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]