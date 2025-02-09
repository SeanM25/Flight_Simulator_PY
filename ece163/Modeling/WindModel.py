import math
import random
from ..Containers import States
from ..Utilities import MatrixMath
from ..Constants import VehiclePhysicalConstants as VPC


class WindModel:



   def __init__(self, dT= 0.01, Va = 25.0, drydenParameters = VPC.DrydenNoWind):

    '''
    function to initialize the wind model code. Will load the appropriate constants that parameterize the wind gusts from the Dryden gust model. 
    Creates the discrete transfer functions for the gust models that are used to update the local wind gusts in the wind frame.
    These are added to the inertial wind (Wn, We, Wd) that are simply constants. Discrete models are held in self and used in the Update function.
    '''
      

    self.dT = dT # Set Time Step for Wind Model to given time step 0.01 by default

    self.Va = Va # Initialize Airspeed with given airspeed 25 m/s by defauly

    self.drydenParameters = drydenParameters # Intialize dryden parameters with the given dryden parmaeters which are windless by default

    self.wind = States.windState(); # Initialize the Wind State

    # Set Initial States for Discrete Time sampling x-

    x_u_prev = [[0]] # Initialize previous state x_u to zero given in lecture

    x_v_prev = [[0], [0]] # Initialize previous state x_v to zero given in lecture

    x_w_prev = [[0], [0]] # Initialize previous state x_w to zero given in lecture

    
    # Intialize u, v, w matrices and set to zero intially since nothing has yet been sampled 

    self.Phi_u = [[0]] # Intialize Phi_u matrix to zero as seen in Dryden handout

    self.Gamma_u = [[0]] # Intialize Gamma_u matrix to zero as seen in Dryden handout

    self.H_u = [[0]] # Intialize H_u matrix to zero as seen in Dryden handout

    # Intialize v matrices

    self.Phi_v = [[0, 0], [0, 0]] # Intialize Phi_v matrix to zero as seen in Dryden handout

    self.Gamma_v = [[0], [0]] # Intialize Gamma_v matrix to zero as seen in Dryden handout

    self.H_v = [[0, 0]] # Intialize H_v matrix to zero as seen in Dryden handout


    # Intialize w matrices which are the same as v

    self.Phi_w = [[0, 0], [0, 0]] # Intialize Phi_v matrix to zero as seen in Dryden handout

    self.Gamma_w = [[0], [0]] # Intialize Gamma_v matrix to zero as seen in Dryden handout

    self.H_w = [[0, 0]] # Intialize H_v matrix to zero as seen in Dryden handout

    # Create Transfer Functions

    WindModel.CreateDrydenTransferFns(self, dT, Va, drydenParameters) # Call function and create the Dryden Trans Functions

    return # Return nothing
   

   def CreateDrydenTransferFns(self, dT, Va, drydenParameters):
     
    '''Function creates the Dryden transfer functions in discrete form. 
    These are used in generating the gust models for wind gusts (in wind frame). 
    If the input is DrydenNoWind, then set Phi_[u,v,w] to Identity, Gamma_{u,v,w] to zero, and H_[u,v,w] to a column of ones. 
    If input Va is less than or 0, raise an arithmetic error since your wind models are undefined for zero or negative airspeeds.'''


    if(Va <= 0): # If the airspeed is zero or negative we throw an error
      
      raise ArithmeticError("Va is Undefined!!! Try Again") # Throw a warning message to the user
    


    # Check Windless Condition


    # Proceed Normally and 









    return # Return nothing