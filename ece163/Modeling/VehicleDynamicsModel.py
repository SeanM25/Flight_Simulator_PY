import math
from ..Containers import States
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC



class VehicleDynamicsModel:


    def __init__(self, dT = VPC.dT):

        '''Initializes the class, and sets the time step (needed for Rexp and integration). Instantiates attributes for vehicle state, and time derivative of vehicle state.'''

        self.dT = dT # Assign time step as the given dT in VPC

        self.state = States.vehicleState() # Instantiates state as an instance of States.vehicleState()

        self.dot = States.vehicleState() # Instantiates dot (time derivative) as an instance of States.vehicleState()

        return


    def getVehicleDerivative(self):

        '''Getter method to read the vehicle state time derivative'''

        return self.dot # Return dot (time state derivative of vehicle)
    
    
    
    




