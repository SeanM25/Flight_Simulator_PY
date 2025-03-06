import sys
sys.path.append("..") #python is horrible, no?


import math
import random
import matplotlib.pyplot as plt

import ece163.Utilities.MatrixMath as mm

import ece163.Constants.VehiclePhysicalConstants as VPC

import numpy as np

# HW 6 Prob 1c.)



T_tot = 10 # Total time for sim

dT = 0.01


TC_1 = 0.1 # Time constant 1

TC_2 = 1.0 # Time constant 2

TC_3 = 10.0 # Time constant 3

a_1 = 2 * math.pi * TC_1 # a with TC 1

a_2 = 2 * math.pi * TC_2 # a with TC 2

a_3 = 2 * math.pi * TC_3 # a with TC 3

num_steps = int(T_tot / dT)

exp_1 = math.exp(-a_1 * dT)

exp_2 = math.exp(-a_2 * dT)

exp_3 = math.exp(-a_3 * dT)


t = [i * dT for i in range(num_steps)]

# Initalize plots for the 4 input signals

input_1 = [0 for i in range(num_steps)]

input_2 = [0 for i in range(num_steps)]

input_3 = [0 for i in range(num_steps)]

input_3 = [0 for i in range(num_steps)]

input_4 = [0 for i in range(num_steps)]


# Initalize all graphs for the 1st signal and the 3 time constants

yk_in1_1 = [0 for i in range(num_steps)]

yk_in1_2 = [0 for i in range(num_steps)]

yk_in1_3 = [0 for i in range(num_steps)]

yk_in1_1_prev = 0.0

yk_in1_2_prev = 0.0

yk_in1_3_prev = 0.0

# Initalize all graphs for the 2nd signal and the 3 time constants

yk_in2_1 = [0 for i in range(num_steps)]

yk_in2_2 = [0 for i in range(num_steps)]

yk_in2_3 = [0 for i in range(num_steps)]

yk_in2_1_prev = 0.0

yk_in2_2_prev = 0.0

yk_in2_3_prev = 0.0

# Initalize all graphs for the 3rd signal and the 3 time constants

yk_in3_1 = [0 for i in range(num_steps)]

yk_in3_2 = [0 for i in range(num_steps)]

yk_in3_3 = [0 for i in range(num_steps)]

yk_in3_1_prev = 0.0

yk_in3_2_prev = 0.0

yk_in3_3_prev = 0.0

# Initalize all graphs for the 4th signal and the 3 time constants

yk_in4_1 = [0 for i in range(num_steps)]

yk_in4_2 = [0 for i in range(num_steps)]

yk_in4_3 = [0 for i in range(num_steps)]

yk_in4_1_prev = 0.0

yk_in4_2_prev = 0.0

yk_in4_3_prev = 0.0

# Graph All signals and filters

for i in range(num_steps):

    # Compute and Graph the 4 signals

    input_1[i] = math.sin(2 * math.pi * t[i])

    input_2[i] = math.sin(20 * math.pi * t[i])

    input_3[i] = (math.sin(2 * math.pi * t[i]) * (math.cos(20 * t[i]) * math.sin(10 * t[i])))

    input_4[i] = (math.sin(2 * math.pi * t[i]) + (math.cos(20 * t[i]) * math.sin(10 * t[i])))

    # Input 1 and the time consts

    yk_in1_1[i] = (exp_1 * yk_in1_1_prev) + ((1 - exp_1) * input_1[i])

    yk_in1_1_prev = yk_in1_1[i]

    yk_in1_2[i] = (exp_2 * yk_in1_2_prev) + ((1 - exp_2) * input_1[i])

    yk_in1_2_prev = yk_in1_2[i]

    yk_in1_3[i] = (exp_1 * yk_in1_3_prev) + ((1 - exp_3) * input_1[i])

    yk_in1_3_prev = yk_in1_3[i]


    # Input 2 and the time constants

    yk_in2_1[i] = (exp_1 * yk_in2_1_prev) + ((1 - exp_1) * input_2[i])

    yk_in2_1_prev = yk_in2_1[i]

    yk_in2_2[i] = (exp_2 * yk_in2_2_prev) + ((1 - exp_2) * input_2[i])

    yk_in2_2_prev = yk_in2_2[i]

    yk_in2_3[i] = (exp_1 * yk_in2_3_prev) + ((1 - exp_3) * input_2[i])

    yk_in2_3_prev = yk_in2_3[i]

    # Input 3 and the time constants

    yk_in3_1[i] = (exp_1 * yk_in3_1_prev) + ((1 - exp_1) * input_3[i])

    yk_in3_1_prev = yk_in3_1[i]

    yk_in3_2[i] = (exp_2 * yk_in3_2_prev) + ((1 - exp_2) * input_3[i])

    yk_in3_2_prev = yk_in3_2[i]

    yk_in3_3[i] = (exp_1 * yk_in3_3_prev) + ((1 - exp_3) * input_3[i])

    yk_in3_3_prev = yk_in3_3[i]

    # Input 4 and the time constants

    yk_in4_1[i] = (exp_1 * yk_in4_1_prev) + ((1 - exp_1) * input_4[i])

    yk_in4_1_prev = yk_in4_1[i]

    yk_in4_2[i] = (exp_2 * yk_in4_2_prev) + ((1 - exp_2) * input_4[i])

    yk_in4_2_prev = yk_in4_2[i]

    yk_in4_3[i] = (exp_1 * yk_in4_3_prev) + ((1 - exp_3) * input_4[i])

    yk_in4_3_prev = yk_in4_3[i]





plt.close("all")

fig = plt.figure()

# Plot input 1, and the filter witheach of the time constants

ax1 = plt.subplot(3,4,1) 

ax1.plot(t, yk_in1_1)

ax1.plot(t, input_1)

ax2 = plt.subplot(3,4,5) 

ax2.plot(t, yk_in1_2)

ax2.plot(t, input_1)

ax3 = plt.subplot(3,4,9) 

ax3.plot(t, yk_in1_3)

ax3.plot(t, input_1)

# Plot input 2, and the filter with each of the time constants

ax4 = plt.subplot(3,4,2) 

ax4.plot(t, yk_in2_1)

ax4.plot(t, input_2)

ax5 = plt.subplot(3,4,6) 

ax5.plot(t, yk_in2_2)

ax5.plot(t, input_2)

ax6 = plt.subplot(3,4,10) 

ax6.plot(t, yk_in2_3)

ax6.plot(t, input_2)

# Plot input 3, and the filter with each of the time constants

ax7 = plt.subplot(3,4,3) 

ax7.plot(t, yk_in3_1)

ax7.plot(t, input_3)

ax8 = plt.subplot(3,4,7) 

ax8.plot(t, yk_in3_2)

ax8.plot(t, input_3)

ax9 = plt.subplot(3,4,11) 

ax9.plot(t, yk_in3_3)

ax9.plot(t, input_3)

# Plot input 4, and the filter with each of the time constants

ax10 = plt.subplot(3,4,4) 

ax10.plot(t, yk_in4_1)

ax10.plot(t, input_4)

ax11 = plt.subplot(3,4,8) 

ax11.plot(t, yk_in4_2)

ax11.plot(t, input_4)

ax12 = plt.subplot(3,4,12) 

ax12.plot(t, yk_in4_3)

ax12.plot(t, input_4)


plt.show()





