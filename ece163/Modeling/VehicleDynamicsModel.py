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

        mtrx_pqr = [[state.p], [state.q] , [state.r]] # pqr (angular rates) matrix

        mtrx_prq_deriv = [[dot.p], [dot.q] , [dot.r]] # pqr dot (angular rates derivative) matrix

        omega_B_I_new = mm.add(mtrx_pqr, mm.scalarMultiply((dT / 2), mtrx_prq_deriv)) # This should correspond to equation (38) in the Attitude Cheat Sheet Also in lecture gets new [[p], [q], [r]]

        p_cur = omega_B_I_new[0][0] # Get present value of p (change in yaw)

        q_cur = omega_B_I_new[1][0] # Get present value of q (change in pitch)

        r_cur = omega_B_I_new[2][0] # Get present value of r (change in roll)

        skew_sym_mtrx = mm.skew(p_cur, q_cur, r_cur) # create skew symmetic matrix from the current values of p, q, and r

        mag_W = math.hypot(p_cur, q_cur, r_cur) # Gets the magnitude ||W|| from the skew symmetric matrix


        I_matrix = [[1, 0, 0], [0, 1, 0], [0, 0, 1]] # Standard identity matrix for finding R exp

        if mag_W <= 0.2: # If ||W|| is between 0 - 0.2 we use the approximations

            exp_SIN_term = (dT) - (((dT ** 3) * (mag_W ** 2)) / (6)) + (((dT ** 5) * (mag_W ** 4)) / (120)) # Equation 27 from Attitutde cheat sheet

            exp_COS_term = ((dT ** 2) / 2) - (((dT ** 4) * (mag_W ** 2)) / (24)) + (((dT ** 6) * (mag_W ** 4)) / (720)) # Equation 28 from Attitutde cheat sheet


        else: # Otherwise

            exp_SIN_term = (math.sin(mag_W * dT) / (mag_W)) # Use Sin term as normal

            exp_COS_term = ((1 - math.cos(mag_W * dT)) / (mag_W ** 2)) # Use Cos term as normal


    # Add everything up to get R_{exp}

        square_skew = mm.multiply(skew_sym_mtrx, skew_sym_mtrx) # Square the skew symmetric matrix
    
        sin_times_w = mm.scalarMultiply(exp_SIN_term,skew_sym_mtrx) # Multiplies Sin term by the skew symmentric

        I_minus_Sin = mm.subtract(I_matrix, sin_times_w) # Subtracts the identity matrix from the Sin term multiplied by the skew symmetric

        cos_times_w_square = mm.scalarMultiply(exp_COS_term, square_skew) # multiplies the cos term by the skew symmetric matrix squared

        Rexp = mm.add(I_minus_Sin, cos_times_w_square) # Add the results to get Rexp

        return Rexp # Return Rexp
    
    def derivative(self, state, forcesMoments):

        '''Function to compute the time-derivative of the state given body frame forces and moments'''

        # Derivatives NED (Pn, Pe, Pd)

        velocity_vector = [[state.u], [state.v], [state.w]] # Get u, v, w matrix

        R_transp = mm.transpose(state.R) # Transpose R inertial to body

        dot_pos_vector = mm.multiply(R_transp, velocity_vector) # multiply R trans by u,v,w to get the position derivative

        pn_dot = dot_pos_vector [0][0] # Get Pn dot

        pe_dot = dot_pos_vector [1][0] # Get Pe dot

        pd_dot = dot_pos_vector [2][0] # Get Pd dot

        # Derivatives of velocities (u, v, w)

        minus_wx_times_uvw = [[(state.r * state.v) - (state.q * state.w)], [(state.p * state.w) - (state.r * state.u)], [(state.q * state.u) - (state.p * state.v)]] # Given expanded matrix from lecture

        F_over_m = [[(forcesMoments.Fx) / (VPC.mass)] , [(forcesMoments.Fy) / (VPC.mass)], [(forcesMoments.Fz) / (VPC.mass)]] # Forces vector scaled by 1/m

        dot_UVW = mm.add(F_over_m, minus_wx_times_uvw) # derivative of the velocities vector

        u_dot = dot_UVW[0][0] # u dot
    
        v_dot = dot_UVW[1][0] # v dot

        w_dot = dot_UVW[2][0] # w dot




        # derivatives of yaw, pitch, roll

        yaw_pitch_roll = [[state.p], [state.q], [state.r]] # Given values of roll, pitch, and yaw

        YPR_dir_mtrx = [[1, (math.sin(state.roll) * math.tan(state.pitch)), (math.cos(state.roll) * math.tan(state.pitch))],
                        [0, math.cos(state.roll), -1 * math.sin(state.roll)],
                        [0, (math.sin(state.roll) / math.cos(state.pitch)), (math.cos(state.roll) / math.cos(state.pitch))]] # Given matrix from lecture
        
        dot_YPR = mm.multiply(YPR_dir_mtrx, yaw_pitch_roll) # get yaw pitch and roll 
        
        pitch_dot = dot_YPR[0][0] # Get pitch dot

        roll_dot = dot_YPR[1][0] # Get roll dot
        
        yaw_dot = dot_YPR[2][0] # get yaw dot

        # Derivitive of UVW

        m_xyz = [[forcesMoments.Mx], [forcesMoments.My], [forcesMoments.Mz]] # Get current moments vector
        
        omega_cross = mm.skew(state.p, state.q, state.r) # skew symmetric

        pqr = [[state.p], [state.q], [state.r]] # get current pqr

        neg_J_inv = mm.scalarMultiply(-1, VPC.JinvBody) # Get -J^-1

        left_term = mm.multiply(VPC.JinvBody, m_xyz) # Multiply J times moments vector

        right_term = mm.multiply(mm.multiply(neg_J_inv, omega_cross), mm.multiply(VPC.Jbody, pqr)) # Multiply the other 4 terms together

        dot_pqr = mm.add(left_term, right_term) # Add them together

        p_dot = dot_pqr[0][0] # Get p dot

        q_dot = dot_pqr[1][0] # get q dot

        r_dot = dot_pqr[2][0] # get r dot

        # Derivative of R

        R_dot = mm.scalarMultiply(-1, (mm.multiply(omega_cross, state.R)))

        dot = States.vehicleState(pn_dot, pe_dot, pd_dot, u_dot, v_dot, w_dot, yaw_dot, roll_dot, pitch_dot, p_dot, q_dot, r_dot)

        dot.R = R_dot # return derivative of the DCM R

        return dot # return all of the state derivatives
    
    def IntegrateState (self, dT, state, dot):


        # Integrate Pn, Pe, Pd using the forward integration formula x_{k+1} = x_{k} + dx/dt * dT

        pn_int = state.pn + (dot.pn * dT)

        pe_int = state.pe + (dot.pe * dT)

        pd_int = state.pd + (dot.pd * dT)

        
        # Integrate u, v, w using the forward integration formula x_{k+1} = x_{k} + dx/dt * dT

        u_int = state.u + (dot.u * dT)

        
        v_int = state.v + (dot.v * dT)

        
        w_int = state.w + (dot.w * dT)

        # Integrate p, q, r using the forward integration formula x_{k+1} = x_{k} + dx/dt * dT

        p_int = state.p + (dot.p * dT)

        q_int = state.q + (dot.q * dT)

        r_int = state.r + (dot.r * dT)

        # Integrate R using matrix exponetial

        Rexp = VehicleDynamicsModel.Rexp(self, dT, state, dot)

        R_int = mm.multiply(Rexp, state.R)

        # Integrate yaw, pitch, roll using R_{k+1}

        yaw_int, pitch_int, roll_int = Rotations.dcm2Euler(R_int)

        newState = States.vehicleState(pn_int, pe_int, pd_int, u_int, v_int, w_int, yaw_int, pitch_int, roll_int, p_int, q_int, r_int, R_int)

        newState.Va = state.Va # Copy Va

        newState.alpha = state.alpha # Copy alpha

        newState.beta = state.beta # Copy beta

        newState.chi = math.atan2(dot.pe, dot.pn) # Get chi using the formula

        return newState

    def ForwardEuler(self, dT, state, dot):

        newState = VehicleDynamicsModel.IntegrateState(self, dT, state, dot) # Do the integration

        return newState # return the new state
    
    def Update(self, forcesMoments):

        self.dot = self.derivative(self.state, forcesMoments) # Get derivative of the state
       
        self.state = self.IntegrateState(self.dT, self.state, self.dot) # Then integrate the state

        return



