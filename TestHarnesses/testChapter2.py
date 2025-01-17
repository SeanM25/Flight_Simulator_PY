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

def compareDCM(a, b): # Quick funtion to compare DCM's
	"""A quick tool to compare two DCM's"""
	DCM_close = [isclose(a[i][j], b[i][j]) for i in range(2) for j in range(2)]
	return all(DCM_close)

def compareTUPLE(a,b): # Quick function to compare tuples
	
	if([isclose(a[i], b[i]) for i in range(2)]):

		return True
	
	else:

		return False


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

ned_points_T1 = [[1, 2, 3], # Random points assumed to be in NED for testing (Test 1)
			 [4, 5, 6]]

ned_points_T2 = [[5, 2, 7], # More Random points assumed to be in NED for testing (Test 2)
			 [4, 3, 8],
			 [7, 3, 2]]

ned_points_T3 = [[2, 4, 6]] # Another set of points assumed to be in NED for testing (Test 3)

ned_T1_expect = [[2, 1, -3],  # Expected ENU conversion for Test 1
			 [5, 4, -6]]

ned_T2_expect = [[2, 5, -7], # Expected ENU conversion for Test 2
			 [3, 4, -8],
			 [3, 7, -2]]

ned_T3_expect = [[4, 2, -6]] # Expected ENU conversion for Test 3

ned_T1_act = Rotations.ned2enu(ned_points_T1) # Actual result of test 1 ned2enu()

ned_T2_act = Rotations.ned2enu(ned_points_T2) # Actual result of test 2 ned2enu()

ned_T3_act = Rotations.ned2enu(ned_points_T3) # Actual result of test 3 ned2enu()

cur_TEST_NED = "Test 1 ned2enu()" # Test 1 name

evaluateTest(cur_TEST_NED, ned_T1_expect == ned_T1_act) # Test 1 Pass ?

cur_TEST_NED = "Test 2 ned2enu()" # Test 2 name

evaluateTest(cur_TEST_NED, ned_T2_expect == ned_T2_act) # Test 2 Pass ?

cur_TEST_NED = "Test 3 ned2enu()" # Test 3 name

evaluateTest(cur_TEST_NED, ned_T3_expect == ned_T3_act) # Test 3 Pass ?


#%% euler2DCM(): 

print("Beginning testing of Rotations.euler2DCM():")

e2DCM_T1_expect = [[(-math.sqrt(2) / 4), (-math.sqrt(2) / 4), (-math.sqrt(3) / 2)], [((1/2) - (math.sqrt(3) / 4)), ((-1/2) - (math.sqrt(3) / 4)), (-math.sqrt(2) / 4)], 
				   [((-1/2) - (math.sqrt(3) / 4)), ((1/2) - (math.sqrt(3) / 4)), (math.sqrt(2) / 4)]] # Given as example in HW 0 expected test 1

e2DCM_T1_act = Rotations.euler2DCM(-135*(math.pi / 180), 60*(math.pi / 180), 45*(math.pi / 180)) # Given as example in HW 0 actual test 1

e2DCM_T2_expect = [[1, 0, 0], [0, 1, 0], [0, 0, 1]] # Also an example from HW 0 Expected test 2 euler2dcm()

e2DCM_T2_act = Rotations.euler2DCM(0, 0, 0) # Also an example in HW 0 actual test 2 euler2dcm()

e2DCM_T3_expect = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]] # Expected reults of test 3 euler2dcm()

e2DCM_T3_act = Rotations.euler2DCM(90*(math.pi / 180), 0, 0) # Actual test 2

cur_TEST_e2DCM = "Test 1 euler2DCM()" # Test 1 name euler2dcm()

evaluateTest(cur_TEST_e2DCM, compareDCM(e2DCM_T1_act,e2DCM_T1_expect)) # Test 1 euler2dcm() pass?

cur_TEST_e2DCM = "Test 2 euler2DCM()" # Test 2 name euler2dcm()


evaluateTest(cur_TEST_e2DCM, compareDCM(e2DCM_T2_act,e2DCM_T2_expect)) # Test 2 euler2dcm() pass?

cur_TEST_e2DCM = "Test 3 euler2DCM()" # Test 3 name euler2dcm()


evaluateTest(cur_TEST_e2DCM, compareDCM(e2DCM_T3_act,e2DCM_T3_expect)) # Test 3 euler2dcm() pass?


#%% DCM2euler():

DCM = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]] # Given DCM for test 1 dcm2euler() reuse of above test

euler_1_act = Rotations.dcm2Euler(DCM) # Actual result of test 1 dcm2euler()

euler_1_expect = (90*(math.pi / 180),0,0) # Expected test 1 result dcm2euler()

cur_TEST_dcm2eul = "Test 1 dcm2euler()" # Test 1 dcm2euler() name

evaluateTest(cur_TEST_dcm2eul, compareTUPLE(euler_1_expect,euler_1_act)) # Test 1 pass?

cur_TEST_dcm2eul = "Test 2 dcm2euler()" # Test 2 dcm2euler() name

DCM_2 = [[(-math.sqrt(2) / 4), (-math.sqrt(2) / 4), (-math.sqrt(3) / 2)], # DCM for test 2 another reuse
		 [((1/2) - (math.sqrt(3) / 4)), ((-1/2) - (math.sqrt(3) / 4)), (-math.sqrt(2) / 4)], [((-1/2) - (math.sqrt(3) / 4)), ((1/2) - (math.sqrt(3) / 4)), (math.sqrt(2) / 4)]] 

euler_2_act = Rotations.dcm2Euler(DCM_2) #Actual result test 2

euler_2_expect = ((-135*(math.pi / 180)), (60*(math.pi / 180)), (45*(math.pi / 180))) # expected result of test 2

evaluateTest(cur_TEST_dcm2eul, compareTUPLE(euler_2_act, euler_2_expect)) #Test 2 Pass ?

cur_TEST_dcm2eul = "Test 3 dcm2euler()" # Test 3 name?

DCM_3 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]] # DCM for test 3

euler_3_act = Rotations.dcm2Euler(DCM_3) # actual test 3 result

euler_3_expect = (0, 0, 0) # expected Test 3 result

evaluateTest(cur_TEST_dcm2eul, compareTUPLE(euler_3_act,euler_3_expect)) # Test 3 pass ?

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