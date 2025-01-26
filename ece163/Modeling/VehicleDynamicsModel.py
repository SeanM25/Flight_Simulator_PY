import math
from ..Containers import States
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

# Author: Sean M. Manger (smanger@ucsc.edu)

# Assignment: Lab 1: Vehicle Dynamics Module

class VehicleDynamicsModel:


    def __init__(self, dT = VPC.dT):

        '''Initializes the class, and sets the time step (needed for Rexp and integration). Instantiates attributes for vehicle state, and time derivative of vehicle state.'''

        self.dT = dT # Assign time step as the given dT in VPC

        self.state = States.vehicleState() # Instantiates state as an instance of States.vehicleState()

        self.dot = States.vehicleState() # Instantiates dot (time derivative) as an instance of States.vehicleState()

        return # Return nothing


    def getVehicleDerivative(self):

        '''Getter method to read the vehicle state time derivative'''

        return self.dot # Return dot (time state derivative of vehicle)
    

    
    def getVehicleState(self):

        '''Getter method to read the vehicle state'''

        return self.state # Return state of aircraft

    def reset(self):

        '''Reset the Vehicle state to initial conditions'''

        self.state = States.vehicleState() # Reset vehicle state to initial

        self.dot = States.vehicleState() # Reset state derivative as well I presume?

        # dT doesn't change so no need to reset it

        return # Return nothing

    def setVehicleDerivative(self, dot):

        '''Setter method to write the vehicle state time derivative'''

        self.dot = dot # set current time derivative to given time derivative

        return # return nothing

    def setVehicleState(self, state):

        '''Setter method to write the vehicle state'''

        self.state = state # set vehicle state to the given state

        return # return nothing
    
    def Rexp(self, dT, state, dot):

        '''Calculates the matrix exponential exp(-dT*[omega x]), which can be used in the closed form solution for the DCM integration from body-fixed rates.'''

        mtrx_pqr = [[self.state.p], [self.state.q] , [self.state.r]] # pqr (angular rates) matrix

        mtrx_prq_deriv = [[self.dot.p], [self.dot.q] , [self.dot.r]] # pqr dot (angular rates derivative) matrix

        omega_B_I_new = mm.add(mtrx_pqr, mm.scalarMultiply(mtrx_prq_deriv, (dT / 2))) # This should correspond to equation (38) in the Attitude Cheat Sheet Also in lecture gets new [[p], [q], [r]]

        p_cur = omega_B_I_new[0][0] # Get present value of p (change in yaw)

        q_cur = omega_B_I_new[1][0] # Get present value of q (change in pitch)

        r_cur = omega_B_I_new[2][0] # Get present value of r (change in roll)

        skew_sym_mtrx = mm.skew(p_cur, q_cur, r_cur) # create skew symmetic matrix from the current values of p, q, and r

        mag_W = math.hypot(p_cur, q_cur, r_cur) # Gets the magnitude ||W|| from the skew symmetric matrix


        I_matrix = [[1,0,0], [0, 1, 0], [0, 0, 1]] # Standard identity matrix for finding R exp

        if mag_W <= 0.2: # If ||W|| is between 0 - 0.2 we use the approximations

            exp_SIN_term = (dT) - (((dT ** 3) * (mag_W ** 2)) / (6)) + (((dT ** 5) * (mag_W ** 4)) / (120)) # Equation 27 from Attitutde cheat sheet

            exp_COS_term = ((dT ** 2) / 2) - (((dT ** 4) * (mag_W ** 2)) / (24)) + (((dT ** 6) * (mag_W ** 4)) / (720)) # Equation 28 from Attitutde cheat sheet


        else: # Otherwise

            exp_SIN_term = (math.sin(mag_W * dT) / (mag_W)) # Use Sin term as normal

            exp_COS_term = ((1 - math.cos(mag_W * dT)) / (mag_W ** 2)) # Use Cos term as normal


    # Add everything up to get R_{exp}

        square_skew = mm.multiply(skew_sym_mtrx, skew_sym_mtrx) # Square the skew symmetric matrix
    
        sin_times_w = mm.scalarMultiply(skew_sym_mtrx, exp_SIN_term) # Multiplies Sin term by the skew symmentric

        I_minus_Sin = mm.subtract(I_matrix, sin_times_w) # Subtracts the identity matrix from the Sin term multiplied by the skew symmetric

        cos_times_w_square = mm.scalarMultiply(exp_COS_term, square_skew) # multiplies the cos term by the skew symmetric matrix squared

        Rexp = mm.add(I_matrix, cos_times_w_square) # Add the results to get Rexp

        return Rexp # Return Rexp



