import sys
sys.path.append("..") #python is horrible, no?


import math
import random
import matplotlib.pyplot as plt

import ece163.Utilities.MatrixMath as mm

import ece163.Constants.VehiclePhysicalConstants as VPC

# Hw 6 Problem 3 Part 3c.) Basic Airspeed Complementary Filter w/ Slower Va dot Faster Va

# That is when we don't have an accelerometer reading we set it to zero and update Va estimate anyways


K_i = 0.1 # Given Ki

K_p = 1 # Given Kp

T_tot  = 10 # Simulate for 10 seconds

f_pitot = 100

f_acc = 10

# Change Timestep to account for Va dot at 10 Hz and Va pitot at 100 Hz

dT_actual = 1 / (math.lcm(f_pitot, f_acc))

num_steps = int(T_tot / dT_actual) # Get Number of steps

# Intitalize All Data

t_data = [i * dT_actual for i in range (num_steps)] # Get necessary time data

Va_est = [0 for i in range(num_steps)] # Empty Airspeed estimate to fill

bias_Va_est = [0 for i in range(num_steps)] # Airspeed bia estimate to fill

Va_pitot = [0 for i in range(num_steps)] # Pitot Data

Va_dot = [0 for i in range(num_steps)] # Accelerometer date

bias_Va_dot = [0 for i in range(num_steps)] # Airspeed bias dot data

Va_est_dot = [0 for i in range(num_steps)] # Airspeed estimate dot data

Va_est = [0 for i in range(num_steps)] # Airspeed estimate data

for i in range(num_steps):

    # This implements the algorithim outlined in the estimation handout with the if measurment condition

    # This is the Airspeed Filter with slower Va dot faster Va

    Va_pitot[i] = 24 + math.sin(2 * math.pi * t_data[i]) # Get pitot data

    Va_dot[i] = 2 * math.pi * math.cos(2 * math.pi * t_data[i]) # Get Accelerometer data

    if(Va_dot[i] > 0): # If we have a measurement that is the accleromoter Va dot does not read 0

        # Update Everything as usual

        bias_Va_dot[i] = (-K_i * (Va_pitot[i] - Va_est[i])) # Get bias estimate derivative

        bias_Va_est[i] = bias_Va_est[i] + (bias_Va_dot[i] * dT_actual) # integrate to get Bias estimate

        Va_est_dot[i] = Va_dot[i] - bias_Va_est[i] + (K_p * (Va_pitot[i] - Va_est[i])) # Get airspeed estimate derivative

        Va_est[i] = Va_est[i] + (Va_est_dot[i] * dT_actual) # Integrate to get Airspeed Estimate
    
    else: # Otherwise if we don't have an accleometer reading set Va dot to zero in between measurements

        Va_dot[i] = 0.0 # Set present accelerometer reading to 0

        bias_Va_dot[i] = (-K_i * (Va_pitot[i] - Va_est[i])) # Get bias estimate derivative

        bias_Va_est[i] = bias_Va_est[i] + (bias_Va_dot[i] * dT_actual) # integrate to get Bias estimate

        Va_est_dot[i] = Va_dot[i] - bias_Va_est[i] + (K_p * (Va_pitot[i] - Va_est[i])) # Get airspeed estimate derivative

        Va_est[i] = Va_est[i] + (Va_est_dot[i] * dT_actual) # Integrate to get Airspeed Estimate


fig, (f1,f2) = plt.subplots(2,1)

f1.plot(t_data, Va_est, label = "Airspeed Estimate")

f1.set_xlabel("time (s)")

f1.set_ylabel("Airspeed (Knots)")

f1.legend()

f2.plot(t_data, bias_Va_est, label = "Airspeed Sensor Bias Estimate")

f2.set_xlabel("time (s)")

f2.set_ylabel("Bias Estimate (V)")

f2.legend()



plt.show()