import math
import random
from ..Containers import States
from ..Utilities import MatrixMath as mm
from ..Constants import VehiclePhysicalConstants as VPC


class WindModel:



   def __init__(self, dT = 0.01, Va = 25.0, drydenParameters = VPC.DrydenNoWind):

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

    #self.Phi_u = [[0]] # Intialize Phi_u matrix to zero as seen in Dryden handout

    #self.Gamma_u = [[0]] # Intialize Gamma_u matrix to zero as seen in Dryden handout

    #self.H_u = [[0]] # Intialize H_u matrix to zero as seen in Dryden handout

    # Intialize v matrices

    #self.Phi_v = [[0, 0], [0, 0]] # Intialize Phi_v matrix to zero as seen in Dryden handout

    #self.Gamma_v = [[0], [0]] # Intialize Gamma_v matrix to zero as seen in Dryden handout

   # self.H_v = [[0, 0]] # Intialize H_v matrix to zero as seen in Dryden handout


    # Intialize w matrices which are the same as v

    #self.Phi_w = [[0, 0], [0, 0]] # Intialize Phi_w matrix to zero as seen in Dryden handout

    #self.Gamma_w = [[0], [0]] # Intialize Gamma_ wmatrix to zero as seen in Dryden handout

    #self.H_w = [[0, 0]] # Intialize H_w matrix to zero as seen in Dryden handout

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

    if(drydenParameters == VPC.DrydenNoWind):

        # U matrices

        self.Phi_u = [[1]] # Set Phi_u matrix to identity as there is no wind

        self.Gamma_u = [[0]] # Set Gamma_u matrix to zero as there is no wind

        self.H_u = [[1]] # Set H_u matrix to col of 1s as there is no wind

        # V matrices

        self.Phi_v = [[1, 0], [0, 1]] # Set Phi_v matrix to identity as there is no wind

        self.Gamma_v = [[0], [0]] # Set Gamma_v matrix to identity as there is no wind

        self.H_v = [[1, 1]] # Set H_v matrix to col of 1s as there is no wind

        # W matrices same as V

        self.Phi_w = [[1, 0], [0, 1]] # Set Phi_w matrix to identity as there is no wind

        self.Gamma_w = [[0], [0]] # Set Gamma_w matrix to identity as there is no wind

        self.H_w = [[1, 1]] # Set H_w matrix to col of 1s as there is no wind


    else:

      # Proceed Normally and and gather the matrices. Note: We just gather the matrices in this function and do the actual sampling and updating states later.

      # Gather Lu, Lv, Lw

        Lu = drydenParameters.Lu # Assign Lu from Dryden parameters

        Lv = drydenParameters.Lv # Assign Lv from Dryden parameters

        Lw = drydenParameters.Lw # Assign Lw from Dryden parmaeters

        # Get necessary sigma values

        sigU = drydenParameters.sigmau # Assign sigma u from Dryden Parameters

        sigV = drydenParameters.sigmav # Assign sigma v from Dryden Parameters

        sigW = drydenParameters.sigmaw # Assign sigma w from Dryden Parameters

        # Calculate Phi, Gamma, H for u

        self.Phi_u = [[math.exp(-1 * (Va / Lu) * dT)]] # Phi u from dryden handout

        self.Gamma_u = [[(Lu / Va) * (1 - (math.exp(-1 * (Va / Lu) * dT)))]] # Gamma u from dryden handout

        self.H_u = [[sigU * math.sqrt((2 * Va) / (math.pi * Lu))]] # H_u from dryden handout


        # Calculate Phi, Gamma, H for v this time we have constants multiplied by matrices

        # Constants / matrices

        exp_v = math.exp(-1 * (Va / Lv) * dT) # exp constant term for v

        sigma_times_sqrt = sigV * math.sqrt((3 * Va) / (math.pi * Lv)) # Other sigma constant term for v

        mtrx_Phi_v = [[1 - ((Va / Lv) * dT), -1 * ((Va/Lv) ** 2) * dT], [dT, 1 + ((Va / Lv) * dT)]] # Matrix needed to calculate Phi v from handout

        mtrx_G_v =[[dT], [(((Lv / Va) ** 2) * (math.exp((Va / Lv) * dT) - 1)) - ((Lv / Va) * dT)]] # Matrix needed to calculate Gamma v from handout

        mtrx_H_v = [[1, Va / (math.sqrt(3) * Lv)]] # # Matrix needed to calculate H v from handout

        # Calculate the Coloring Filters

        self.Phi_v = mm.scalarMultiply(exp_v, mtrx_Phi_v) # Calculate Phi_v

        self.Gamma_v = mm.scalarMultiply(exp_v, mtrx_G_v) # Calculate Gamma_v

        self.H_v = mm.scalarMultiply(sigma_times_sqrt, mtrx_H_v) # Calculate H_v

      # Calculate Phi, Gamma, H for w same as above just repace constants with Lw

        exp_w = math.exp(-1 * (Va / Lw) * dT) # exp constant term for w

        sigma_times_sqrt_w = sigW * math.sqrt((3 * Va) / (math.pi * Lw)) # Other sigma constant term for w

        mtrx_Phi_w = [[1 - ((Va / Lw) * dT), -1 * ((Va/Lw) ** 2) * dT], [dT, 1 + ((Va / Lw) * dT)]] # Matrix needed to calculate Phi w from handout

        mtrx_G_w =[[dT], [(((Lw / Va) ** 2) * (math.exp((Va / Lw) * dT) - 1)) - ((Lw / Va) * dT)]] # Matrix needed to calculate Gamma v from handout

        mtrx_H_w = [[1, Va / (math.sqrt(3) * Lw)]] # # Matrix needed to calculate H v from handout

        # Calculate the Coloring Filters

        self.Phi_w = mm.scalarMultiply(exp_w, mtrx_Phi_w) # Calculate Phi_w

        self.Gamma_w = mm.scalarMultiply(exp_w, mtrx_G_w) # Calculate Gamma_w

        self.H_w = mm.scalarMultiply(sigma_times_sqrt_w, mtrx_H_w) # Calculate H_w

        return # return nothing
   

   def Update(self, uu=None, uv=None, uw=None):

    '''
    Function that updates the wind gusts and inserts them back into the .Wind portion of self. 
    This is done by running white noise [Gaussian(0,1)] through the coloring filters of the Dryden Wind Gust model.
    '''
     
    # This function follows the steps described under "Implementation" of the Dryden Handout

    # Run the Gaussian to mimic wind note: this is Pseudo Random and not truely random. For a Sim it is fine however

    if(uu == None):
      
      uu = random.gauss(0, 1) # Generate Random noise for mu in u

    if(uv == None):
      
      uv = random.gauss(0, 1) # Generate Random noise for mu in v

    if(uw == None):
      
      uw = random.gauss(0, 1) # Generate Random noise for mu in w

    # Get previous states

    Xu_minus = self.x_u_prev

    Xv_minus = self.x_v_prev

    Xw_minus = self.x_w_prev   


    # Begin time sampling for u  

    new_Xu_state = mm.add(mm.multiply(self.Phi_u, Xu_minus), mm.scalarMultiply(uu, self.Gamma_u)) # Equation 2 from Dryden Handout

    Wu_update = mm.multiply(self.H_u, new_Xu_state) # Equation 3 from Dryden Handout

    self.wind.Wu = Wu_update[0][0] # Update current Wu wind parameter

    self.x_u_prev = new_Xu_state # Set old u state to new u state


    # Begin time sampling for v  

    new_Xv_state = mm.add(mm.multiply(self.Phi_v, Xv_minus), mm.scalarMultiply(uv, self.Gamma_v)) # Equation 2 from Dryden Handout

    Wv_update = mm.multiply(self.H_v, new_Xv_state) # Equation 3 from Dryden Handout

    self.wind.Wv = Wv_update[0][0] # Update current Wv wind parameter

    self.x_v_prev = new_Xv_state # Set old v state to new v state

    # Begin time sampling for w  

    new_Xw_state = mm.add(mm.multiply(self.Phi_w, Xw_minus), mm.scalarMultiply(uw, self.Gamma_w)) # Equation 2 from Dryden Handout

    Ww_update = mm.multiply(self.H_w, new_Xw_state) # Equation 3 from Dryden Handout

    self.wind.Ww = Ww_update[0][0] # Update current Ww wind parameter

    self.x_w_prev = new_Xw_state # Set old w state to new w state

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
    
    return self.Phi_u, self.Gamma_u, self.H_u, self.Phi_v, self.Gamma_v, self.H_v, self.Phi_w, self.Gamma_w, self.H_w
   
   # Return all Phi's, Gamma's, and H's from the transfer functions


   
