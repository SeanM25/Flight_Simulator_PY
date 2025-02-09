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

    self.x_u_prev = [[0]] # Initialize previous state x_u to zero given in lecture

    self.x_v_prev = [[0], [0]] # Initialize previous state x_v to zero given in lecture

    self.x_w_prev = [[0], [0]] # Initialize previous state x_w to zero given in lecture

    
    # Intialize u, v, w matrices and set to zero intially since nothing has yet been sampled 

    self.Phi_u = [[0]] # Intialize Phi_u matrix to zero as seen in Dryden handout

    self.Gamma_u = [[0]] # Intialize Gamma_u matrix to zero as seen in Dryden handout

    self.H_u = [[0]] # Intialize H_u matrix to zero as seen in Dryden handout

    # Intialize v matrices

    self.Phi_v = [[0, 0], [0, 0]] # Intialize Phi_v matrix to zero as seen in Dryden handout

    self.Gamma_v = [[0], [0]] # Intialize Gamma_v matrix to zero as seen in Dryden handout

    self.H_v = [[0, 0]] # Intialize H_v matrix to zero as seen in Dryden handout


    # Intialize w matrices which are the same as v

    self.Phi_w = [[0, 0], [0, 0]] # Intialize Phi_w matrix to zero as seen in Dryden handout

    self.Gamma_w = [[0], [0]] # Intialize Gamma_ wmatrix to zero as seen in Dryden handout

    self.H_w = [[0, 0]] # Intialize H_w matrix to zero as seen in Dryden handout

    # Create Transfer Functions

    WindModel.CreateDrydenTransferFns(self, dT, Va, drydenParameters) # Call function and create the Dryden Trans Functions

    return # Return nothing
   

   def CreateDrydenTransferFns(self, dT, Va, drydenParameters):
     
    '''Function creates the Dryden transfer functions in discrete form. 
    These are used in generating the gust models for wind gusts (in wind frame). 
    If the input is DrydenNoWind, then set Phi_[u,v,w] to Identity, Gamma_{u,v,w] to zero, and H_[u,v,w] to a column of ones. 
    If input Va is less than or 0, raise an arithmetic error since your wind models are undefined for zero or negative airspeeds.'''


    if(Va <= 0): # If the airspeed is zero or negative we throw an error
      
      raise ArithmeticError("Va is Undefined!!! Va must be neither negative nor Zero") # Throw a warning message to the user
    

    # Check Windless Condition

    


    # Proceed Normally and and gather the matrices. Note: We just gather the matrices in this function and do the actual sampling and updating states later.

    







    return # Return nothing
   

   def getWind(self):
     
    '''Wrapper function to return the wind state from the module '''

    return self.wind # Return present wind state
   
   def setWind(self, windState):
     
     '''Wrapper function that allows for injecting constant wind and gust values into the class :param windState:
       class from vehicleStates with inertial constant wind and wind frame gusts'''
     
     self.wind = windState # Inject wind and gusts values into the present wind state
     

     return # return nothing
   
   def reset(self):
     
     '''Wrapper function that resets the wind model code (but does not reset the model parameters). 
     This will reset both steady and gust winds to zero, and re-set the internal states of the stochastic Dyden wind model to zero.
       To change the model transfer functions you need to use CreateDrydenTranferFns().'''
     
     # Reset model states

     self.x_u_prev = [[0]] # Reset previous state x_u to zero given in lecture

     self.x_v_prev = [[0], [0]] # Reset previous state x_v to zero given in lecture

     self.x_w_prev = [[0], [0]] # Reset previous state x_w to zero given in lecture

     # Rest Wind Model but not parameters

     self.wind = States.windState() # Rest wind Model but parameters remain at present value
     
     return # Return nothing
   

   def setWindModelParameters(self, Wn=0.0, We=0.0, Wd=0.0, drydenParameters=VPC.DrydenNoWind):

    '''Wrapper function that will inject parameters into the wind model. 
       This class models wind according to the Dryden Wind Model, where winds are a static component plus stocahstically derived gusts.'''
     
    self.wind.Wn = Wn # Inject new inertial northern wind value

    self.wind.We = We # Inject new inertial eastern wind value

    self.wind.Wd = Wd # Inject new inertial downwards wind value

    self.drydenParameters = drydenParameters # inject new dryden parameters

    return # return nothing
   
   def getDrydenTransferFns(self):
     
    '''Wrapper function to return the internals of the Dryden Transfer function in order to be able to test the code without requiring consistent internal names.
      Returns the discretized version of the Drydem gust model as outlined in the ECE163_DrydenWindModel handout (Phi_u, Gamma_u, H_u, Phi_v, Gamma_v, H_v, Phi_w, Gamma_w, H_w).'''
    
    return self.Phi_u, self.Phi_v, self.Phi_w, self.Gamma_u, self.Gamma_v, self.Gamma_w, self.H_u, self.H_v, self.H_w
   
   # Return all Phi's, Gamma's, and H's from the transfer functions


   
