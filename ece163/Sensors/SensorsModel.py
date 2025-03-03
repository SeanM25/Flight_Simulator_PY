import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Utilities import MatrixMath
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel



class GaussMarkov:

    def __init__(self, dT = 0.01, tau = 1.0e6, eta = 0.0):

        # Sets Inital Parameters to Compute the Gauss Markov Model


        self.tau = tau # Get tau parameter

        self.eta = eta # Get eta

        self.dT = dT # get time step

        self.prev_V  = 0.0 # Prev GM state is intitialized to 0

        return # Return nothing
    

    def reset(self):

        # Resets All Gauss Markov Parameters to IC's

        self.dT = 0.01 # reset time step

        self.tau = 1.0e6 # Reset tau

        self.eta = 0.0 # Reset eta

        self.prev_V = 0.0 # Reset prev GM state

        return # return nothing

        

    
        


        




        
