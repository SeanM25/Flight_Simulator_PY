import sys
sys.path.append("..") #python is horrible, no?


import math
import random
import matplotlib.pyplot as plt

import ece163.Utilities.MatrixMath as mm

import ece163.Constants.VehiclePhysicalConstants as VPC


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



# Part 2b. Control Block Diagram with V(t) and yaw added together

a_1 = 0.79 # a_1 value for TF

a_2 = 4.19 # a_2 value for TF

b_2 = 4.19 # b_2 value for TF

K_p = -12 # Kp value (Proprtional Gain)

K_i = -4 # Ki value (Integral Gain)



A = [[-1 * a_1, -1 * a_2], [1, 0]] # Matrix A from State Space

B = [[1], [0]] # Matrix B from State Space

C = [[0 , b_2]] # Matrix C from State Space

yaw_data = [0 for i in range(num_steps)] # Intialize empty yaw_data array

est_yaw_data = [0 for i in range(num_steps)]

rudder_data = [0 for i in range(num_steps)] # Intialize empty rudder_data array

wind_data = [(0 if t < 1 else 10*math.pi / 180) for t in t_data] # Create wind_data array

accumulator = 0.0 # Accumulator is initially 0

ac_MAX = 1 # Accumulator max not really needed


x = [[0], [0]]

for i in range(num_steps):

    yaw_data[i] = mm.multiply(C, x)[0][0]

    err = (0 - yaw_data[i] - V[i]) # Error with Gauss Markov process added in 

    est_yaw_data[i] = yaw_data[i] + V[i] # Estimated yaw data is the yaw data added to the Gauss Markov process

    accumulator += err * dT # accumulator is the error times the time state

    accumulator = max(min(accumulator, ac_MAX), -ac_MAX) # set the accumulator bounds
    
    rudder_data[i] = err * ((K_i * accumulator) + K_p) # rudder data is the error times Ki/s + Kp

    u = wind_data[i] +  VPC.CndeltaR / VPC.Cnbeta * rudder_data[i] # Update u

    x_dot = mm.add( # Calculate x dot

        mm.multiply(A, x),

        mm.scalarMultiply(u, B))
    
    x = mm.add( # Calculate x

        x,

        mm.scalarMultiply(dT, x_dot))
    






plt.close("all")

fig, GM = plt.subplots() # Create a figure to show the gauss markov plot

GM.plot(t_data, V) # Plot gauss markov data

GM.set_xlabel("time (s)")

GM.set_ylabel("Gauss Markov Magnitude (dB)")

#GM.legend()

fig, (f1, f2) = plt.subplots(2, 1) # Plot wind angle and yaw response on same figure

f1.plot(t_data, wind_data, label = "wind angle")

f1.plot(t_data, yaw_data, label = "yaw response")

f1.set_xlabel("time (s)")

f1.set_ylabel("angle (rad)")

f1.legend()


f2.plot(t_data, est_yaw_data, label = "estimated yaw response") # Plot estimated yaw response

f2.set_xlabel("time (s)")

f2.set_ylabel("angle for estimated yaw (rad)")

f2.legend()

plt.show() # Show Gauss Markov plot