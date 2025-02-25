import sys
sys.path.append("..") #python is horrible, no?


import math
import random
import matplotlib.pyplot as plt

import ece163.Utilities.MatrixMath as mm

import ece163.Constants.VehiclePhysicalConstants as VPC

a_1 = 0.79

a_2 = 4.19

b_2 = 4.19

K_p = -12

K_i = -4



A = [[-1 * a_1, -1 * a_2], [1, 0]]

B = [[1], [0]]

C = [[0 , b_2]]

dT = 0.001

T_tot = 16

n_steps = int(T_tot / dT)

t_data = [i * dT for i in range(n_steps)]

yaw_data = [0 for i in range(n_steps)]

rudder_data = [0 for i in range(n_steps)]

wind_data = [(0 if t < 1 else 10*math.pi / 180) for t in t_data]

past_err = 0.0

accumulator = 0.0

ac_MAX = 1

x = [[0], [0]]

for i in range(n_steps):

    yaw_data[i] = mm.multiply(C, x)[0][0]

    err = (0 - yaw_data[i])

    accumulator += err * dT

    accumulator = max(min(accumulator, ac_MAX), -ac_MAX)
    
    rudder_data[i] = err * ((K_i * accumulator) + K_p)

    u = wind_data[i] +  VPC.CndeltaR / VPC.Cnbeta * rudder_data[i] 

    x_dot = mm.add(

        mm.multiply(A, x),

        mm.scalarMultiply(u, B))
    
    x = mm.add(

        x,

        mm.scalarMultiply(dT, x_dot))
    



plt.close("all")

fig, (ax, ax2) = plt.subplots(2, 1)

ax.plot(t_data, wind_data, label = "wind angle")

ax.plot(t_data, yaw_data, label = "yaw response")

ax.set_xlabel("time (s)")

ax.set_ylabel("angle (rad)")

ax.legend()

ax2.plot(t_data, rudder_data, label = "rudder deflection")

ax2.set_xlabel("time (s)")

ax2.set_ylabel("deflection angle (rad)")

ax2.legend()
   
plt.show()