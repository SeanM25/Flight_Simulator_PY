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

def compareDCM(a, b):
	"""A quick tool to compare two DCM's"""
	DCM_close = [isclose(a[i][j], b[i][j]) for i in range(2) for j in range(2)]
	return all(DCM_close)



failed = []
passed = []
def evaluateTest(test_name, boolean):
	"""evaluateTest prints the output of a test and adds it to one of two 
	global lists, passed and failed, which can be printed later"""
	if boolean:
		#print(f"   passed {test_name}")
		passed.append(test_name)
	else:
		#print(f"   failed {test_name}")
		failed.append(test_name)
	return boolean

"""
#%% Euler2dcm():
print("Beginning testing of Rotations.Euler2dcm()")

cur_test = "Euler2dcm yaw test 1"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(90*math.pi/180, 0, 0)
orig_vec = [[1],[0],[0]]
expected_vec = [[0],[-1],[0]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")
"""

#%% ned2enu(): 

print("Beginning testing of Rotations.ned2enu()")

ned_points_T1 = [[1, 2, 3],
			 [4, 5, 6]]

ned_points_T2 = [[5, 2, 7],
			 [4, 3, 8],
			 [7, 3, 2]]

ned_points_T3 = [[2, 4, 6]]

ned_T1_expect = [[2, 1, -3], 
			 [5, 4, -6]]

ned_T2_expect = [[2, 5, -7],
			 [3, 4, -8],
			 [3, 7, -2]]

ned_T3_expect = [[4, 2, -6]]

ned_T1_act = Rotations.ned2enu(ned_points_T1)

ned_T2_act = Rotations.ned2enu(ned_points_T2)

ned_T3_act = Rotations.ned2enu(ned_points_T3)

cur_TEST_NED = "Test 1 ned2enu()"

evaluateTest(cur_TEST_NED, ned_T1_expect == ned_T1_act)

cur_TEST_NED = "Test 2 ned2enu()"

evaluateTest(cur_TEST_NED, ned_T2_expect == ned_T2_act)

cur_TEST_NED = "Test 3 ned2enu()"

evaluateTest(cur_TEST_NED, ned_T3_expect == ned_T3_act)


#%% euler2DCM(): 

print("Beginning testing of Rotations.euler2DCM():")

e2DCM_T1_expect = [[(-math.sqrt(2) / 4), (-math.sqrt(2) / 4), (-math.sqrt(3) / 2)], [((1/2) - (math.sqrt(3) / 4)), ((-1/2) - (math.sqrt(3) / 4)), (-math.sqrt(2) / 4)], [((-1/2) - (math.sqrt(3) / 4)), ((1/2) - (math.sqrt(3) / 4)), (math.sqrt(2) / 4)]] # Given as example in HW 0

e2DCM_T1_act = Rotations.euler2DCM(-135*(math.pi / 180), 60*(math.pi / 180), 45*(math.pi / 180)) # Given as example in HW 0

e2DCM_T2_expect = [[1, 0, 0], [0, 1, 0], [0, 0, 1]] # Also an example from HW 0

e2DCM_T2_act = Rotations.euler2DCM(0, 0, 0) # Also an example in HW 0

e2DCM_T3_expect = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]

e2DCM_T3_act = Rotations.euler2DCM(90*(math.pi / 180), 0, 0)

cur_TEST_e2DCM = "Test 1 euler2DCM()"

evaluateTest(cur_TEST_e2DCM, compareDCM(e2DCM_T1_act,e2DCM_T1_expect))

cur_TEST_e2DCM = "Test 2 euler2DCM()"

evaluateTest(cur_TEST_e2DCM, compareDCM(e2DCM_T2_act,e2DCM_T2_expect))

cur_TEST_e2DCM = "Test 3 euler2DCM()"

evaluateTest(cur_TEST_e2DCM, compareDCM(e2DCM_T3_act,e2DCM_T3_expect))


#%% DCM2euler():

DCM = [[0, 0, 1], [0, -1, 0], [-1, 0, 0]]

Rotations.dcm2Euler(DCM)

"""
Students, add more tests here.  
You aren't required to use the testing framework we've started here, 
but it will work just fine.
"""

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]