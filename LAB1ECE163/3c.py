import sys
sys.path.append("..") #python is horrible, no?


import math
import random
import matplotlib.pyplot as plt

import ece163.Utilities.MatrixMath as mm

import ece163.Constants.VehiclePhysicalConstants as VPC

# Hw 6 Problem 3 Part 3a.) Basic Airspeed Complementary Filter



K_i = 0.1 # Given Ki

K_p = 1 # Given Kp

T_tot  = 10 # Simulate for 10 seconds

f_acc = 10

f_pitot = 100

dT = 0.01 # Given time step

dT_10Hz = 0.1

num_steps = int(T_tot / dT) # Get Number of steps

# Intitalize All Data

t_data = [i * dT for i in range (num_steps)] # Get necessary time data

Va_est = [0 for i in range(num_steps)] # Empty Airspeed estimate to fill

bias_Va_est = [0 for i in range(num_steps)] # Airspeed bia estimate to fill

Va_pitot = [0 for i in range(num_steps)] # Pitot Data

Va_dot = [0 for i in range(num_steps)] # Accelerometer date

bias_Va_dot = [0 for i in range(num_steps)] # Airspeed bias dot data

Va_est_dot = [0 for i in range(num_steps)] # Airspeed estimate dot data

Va_est = [0 for i in range(num_steps)] # Airspeed estimate data

count_10Hz = 0


for i in range(num_steps):

    Va_pitot[i] = 24 + math.sin(2 * math.pi * t_data[i]) # Get pitot data

    count_10Hz += 1

    if(count_10Hz == 10): # Same Idea as, only update every 10Hz when there is a measurement

        count_10Hz = 0

        #print(t_data[i])


        Va_dot[i] = 2 * math.pi * math.cos(2 * math.pi  * t_data[i]) # Get Accelerometer data


        bias_Va_dot[i] = (-K_i * (Va_pitot[i] - Va_est[i])) # Get bias estimate derivative

        bias_Va_est[i] = bias_Va_est[i] + (bias_Va_dot[i] * 0.1) # integrate to get Bias estimate

        Va_est_dot[i] = Va_dot[i] - bias_Va_est[i] + (K_p * (Va_pitot[i] - Va_est[i])) # Get airspeed estimate derivative

        Va_est[i] = Va_est[i] + (Va_est_dot[i] * 0.1) # Integrate to get Airspeed Estimate

    else: # Otherwise set to 0 and continue update

        Va_dot[i] = 0.0 # Get Accelerometer data

        bias_Va_dot[i] = (-K_i * (Va_pitot[i] - Va_est[i])) # Get bias estimate derivative

        bias_Va_est[i] = bias_Va_est[i] + (bias_Va_dot[i] * 0.1) # integrate to get Bias estimate

        Va_est_dot[i] = Va_dot[i] - bias_Va_est[i] + (K_p * (Va_pitot[i] - Va_est[i])) # Get airspeed estimate derivative

        Va_est[i] = Va_est[i] + (Va_est_dot[i] * 0.1) # Integrate to get Airspeed Estimate





    

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

