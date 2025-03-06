import sys
sys.path.append("..") #python is horrible, no?


import math
import random
import matplotlib.pyplot as plt

import ece163.Utilities.MatrixMath as mm

import ece163.Constants.VehiclePhysicalConstants as VPC

#import numpy as np


# Part 2a. Gauss Markov Plot

dT = 0.001 # Given time step

T_tot = 16 # Total plotting time is 16 seconds

mu = 0 # mu is 0

sigma = 0.0013 # given sigma

tau = 400 # given tau value

V_o = 0 # GM has an initial state of 0

num_steps = int (T_tot / dT) # Calculate the number of steps

V = [0 for i in range(num_steps)] # Fill an empty Gauss Markov array 

t_data = [i * dT for i in range (num_steps)] # Get the time value array


for i in range (num_steps): # For every time value
 
 n = random.gauss(mu, sigma) # Pick a random number from the gaussian distribution

 V[i] = math.exp(t_data[i] / tau * -1) * V_o + n # Given equation

 V_o = V[i] # New Previous state is current state



###

a_1 = 0.79 # a_1 value for TF

a_2 = 4.19 # a_2 value for TF

b_2 = 4.19 # b_2 value for TF

K_p = -12 # Kp value (Proprtional Gain)

K_i = -4 # Ki value (Integral Gain)



A = [[-1 * a_1, -1 * a_2], [1, 0]] # Matrix A from State Space

B = [[1], [0]] # Matrix B from State Space

C = [[0 , b_2]] # Matrix C from State Space

yaw_data = [0 for i in range(num_steps)] # Intialize empty yaw_data array

dYaw_dt = [0 for i in range(num_steps)]

ygyro = dYaw_dt = [0 for i in range(num_steps)]

est_yaw_data = [0 for i in range(num_steps)]

rudder_data = [0 for i in range(num_steps)] # Intialize empty rudder_data array

wind_data = [(0 if t < 1 else 10*math.pi / 180) for t in t_data] # Create wind_data array

accumulator = 0.0 # Accumulator is initially 0

accumulator_estYaw = 0.0

ac_MAX = 1 # Accumulator max not really needed

ac_yaw_max = 1


x = [[0], [0]]

for i in range(num_steps):

    yaw_data[i] = mm.multiply(C, x)[0][0] # Get yaw data

    dYaw_dt[i] = ((yaw_data[i] - yaw_data[i-1]) / dT) # Calculate derivative of yaw

    ygyro = V[i] + dYaw_dt[i] # y gyro term equals yaw deriv + gauss markov

    accumulator_estYaw += ygyro * dT # integrate y gyro with accumulator

    accumulator_estYaw = max(min(accumulator_estYaw, ac_yaw_max), -ac_yaw_max) # Check accumulator bounds

    est_yaw_data[i] = accumulator_estYaw # Get estimated yaw data

    err = (0 - est_yaw_data[i]) # Error with Gauss Markov process added in 

    accumulator += err * dT # integrate the error with the second accumulator

    accumulator = max(min(accumulator, ac_MAX), -ac_MAX) # Check accumulator bounds
        
    rudder_data[i] = err * ((K_i * accumulator) + K_p) # get rudder data form hasn't changed

    u = wind_data[i] +  VPC.CndeltaR / VPC.Cnbeta * rudder_data[i]  # update u

    x_dot = mm.add( # get x dot for state sapce

        mm.multiply(A, x),

        mm.scalarMultiply(u, B))
    
    x = mm.add( # get x  for state sapce

        x,

        mm.scalarMultiply(dT, x_dot))
    




plt.close("all")

fig, (f1, f2) = plt.subplots(2, 1) # Plot yaw response w/ GM and integration

f1.plot(t_data, wind_data, label = "wind angle")

f1.plot(t_data, yaw_data, label = "yaw response")

f1.set_xlabel("time (s)")

f1.set_ylabel("angle (rad)")

f1.legend()


f2.plot(t_data, est_yaw_data, label = "estimated yaw response") # Plot estimated yaw response w/ GM and integration

f2.plot(t_data, yaw_data, label = "yaw response")

f2.set_xlabel("time (s)")

f2.set_ylabel("angle for estimated yaw (rad)")

f2.legend()

plt.show() # Show Gauss Markov plot