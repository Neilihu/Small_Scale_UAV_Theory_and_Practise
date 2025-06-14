"""This file is a test harness for the module ece163.Utilities.Rotations,
and for the method ece163.Modeling.VehicleGeometry.getNewPoints(). 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter2.py (from the root directory) -or-
python testChapter2.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the Rotations module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG

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


#%% Euler2dcm():
print("The number need to be 1e-10 accuracy to be considered as =")
print("\nBeginning testing of Rotations.Euler2dcm()")

cur_test = "Euler2dcm yaw test 1"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(90*math.pi/180, 0, 0)
orig_vec = [[1],[0],[0]]
expected_vec = [[0],[-1],[0]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")


#%%

"""
Students, add more tests here.  
You aren't required to use the testing framework we've started here, 
but it will work just fine.
"""
cur_test = "Euler2dcm test simple"
R = Rotations.euler2DCM(0, 0, 0)
act = mm.multiply(R, [[1],[1],[1]])
exp = [[1],[1],[1]]

if not evaluateTest(cur_test, compareVectors(exp, act)):
	print(f"{exp} != {act}")

cur_test = "Euler2dcm test hard"
R = Rotations.euler2DCM(0.5*math.pi, -3/5*math.pi, math.pi)
act = mm.multiply(R, [[1],[1],[1]])
exp = [[0.6420395219], [0.999999999], [1.2600735106]]
if not evaluateTest(cur_test, compareVectors(exp, act)):
	print(f"{exp} != {act}")
#------------------------------------------------------------------
print("\nBeginning testing of Rotations.dcm2Euler()")

cur_test = "dcm2Euler test simple"
act = Rotations.dcm2Euler([[0.707, 0, -0.707],[0, 1, 0],[0.707, 0, 0.707]])
act = mm.transpose([act])
exp = mm.transpose([[0,0.785247163,0]])
if not evaluateTest(cur_test, compareVectors(exp, act)):
	print(f"{exp} != {act}")

cur_test = "dcm2Euler test hard"
act = Rotations.dcm2Euler([[1, 2, 3],[4, 5, -6],[-7, 8, 9]])
act = mm.transpose([act])
exp = mm.transpose([[1.1071487177,-1.5707963267,-0.5880026035]])
if not evaluateTest(cur_test, compareVectors(exp, act)):
	print(f"{exp} != {act}")

#------------------------------------------------------------------
print("\nBeginning testing of Rotations.ned2enu()")

cur_test = "ned2enu test simple"
act = Rotations.ned2enu([[0,0,0]])
act = mm.transpose(act)
exp = mm.transpose([[0,0,0]])
if not evaluateTest(cur_test, compareVectors(exp, act)):
	print(f"{exp} != {act}")

cur_test = "ned2enu test hard"
act = Rotations.ned2enu([[-2,7,5],[6, 0.5, 0]])
act = mm.transpose(act)
exp = mm.transpose([[7,-2,-5],[0.5,6,0]])
if not evaluateTest(cur_test, compareVectors(exp, act)):
	print(f"{exp} != {act}")

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]
