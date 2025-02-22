"""This file is a test harness for the module VehiclePerturbationModels. 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter5.py (from the root directory) -or-
python testChapter5.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehiclePerturbationModels module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Modeling.WindModel as WM
#import ece163.Controls.VehicleTrim as VehicleTrim
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Controls.VehicleClosedLoopControl as VCLC

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


#%% PUT A TEST HERE?

# PD Control Tests

PD_Init = VCLC.PDControl(1, 1, 1, 2, 4)

cur_test_PD = "PD Control Initialization Test"

if(PD_Init.kp == 1 and PD_Init.kd == 1 and PD_Init.trim == 1 and PD_Init.lowLimit == 2 and PD_Init.highLimit == 4):
	
    PD_good = True
	
evaluateTest(cur_test_PD, PD_good)

cur_test_PD = "PD Update Test"

u_act = VCLC.PDControl.Update(PD_Init, 8, 2, 0)

u_expect = 4

if(isclose(u_act,u_expect)):
	
    Update_pass = True
	
evaluateTest(cur_test_PD, Update_pass)    



cur_test_PD = "setPDGains Test"

VCLC.PDControl.setPDGains(PD_Init, 1, 2, 3, 4, 5)

if(PD_Init.kp == 1 and PD_Init.kd == 2 and PD_Init.trim == 3 and PD_Init.lowLimit == 4 and PD_Init.highLimit == 5):
	
    PD_good_again = True
	
evaluateTest(cur_test_PD, PD_good_again)

# PI Tests







#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]