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


        self.tau = tau # Get tau parameter

        self.eta = eta # Get eta

        self.dT = dT # get time step

        self.V = 0.0 # Current GM Value is set to 0

        self.prev_V  = 0.0 # Prev GM state is also 0

        return # Return nothing
    

    
        


        




        
