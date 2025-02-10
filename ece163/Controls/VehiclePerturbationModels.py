import math
from ece163.Modeling import VehicleAerodynamicsModel as VAM
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath

def dThrust_dVa(self, Va, Throttle, epsilon=0.5):

    '''
    Function to calculate the numerical partial derivative of propeller thrust to change in airspeed using the actual prop function
    from complex propeller model (inside the VehicleAerodynamicsModel class)
    '''


    # This function gets the partial derivative of thrust with respect to the propeller this is easily accomplished via the definition from Calc
    # Essentially we add epsilon to the parameter we are taking the derivative with respect to in this case Va

    Fx , Mx = VAM.VehicleAerodynamicsModel.CalculatePropForces(self,Va,Throttle) # Regular Fx Mx not used

    Fx_eps, mx_eps = VAM.VehicleAerodynamicsModel.CalculatePropForces(self, (Va + epsilon), Throttle) # Fx with epsilon added to Va

    dTdVA = (Fx_eps - Fx) / epsilon # Get partial derivative w/ the formula

    return dTdVA # Return partial of Thrus with respect to Va


