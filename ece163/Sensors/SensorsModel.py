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


    def update(self, vnoise = None):

        # Updates and Computes the Gauss Markov function with the option to drive GM with known value vnoise

        tau = self.tau # Get current tau

        dT = self.dT # get current time step

        if (self.eta == None): # If 0 then random.gauss(0,0) which is just 0

            w = 0.0 # w is 0 this is the random number from GM
        else:

            w = random.gauss(0, self.eta) # Otherwise random.gauss(0, eta)

        
        if(vnoise == None): # If were not driving with a known value 

            V = (math.exp(-(dT / tau)) * self.prev_V) + w # Use random gauss

        else:

            V = (math.exp(-(dT / tau)) * self.prev_V) + vnoise # otherwise use Vnoise

        self.prev_V = V # set previous GM state to current GM state

        return V # Return GM


class GaussMarkovXYZ:





        

    
        


        




        
