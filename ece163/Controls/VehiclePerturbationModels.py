import math
from ece163.Modeling import VehicleAerodynamicsModel as VAM
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath

def dThrust_dVa(Va, Throttle, epsilon=0.5):

    '''
    Function to calculate the numerical partial derivative of propeller thrust to change in airspeed using the actual prop function
    from complex propeller model (inside the VehicleAerodynamicsModel class)
    '''


    # This function gets the partial derivative of thrust with respect to the propeller this is easily accomplished via the definition from Calc
    # Essentially we add epsilon to the parameter we are taking the derivative with respect to in this case Va

    Fx , Mx = VAM.VehicleAerodynamicsModel.CalculatePropForces(Va,Throttle) # Regular Fx Mx not used

    Fx_eps, mx_eps = VAM.VehicleAerodynamicsModel.CalculatePropForces((Va + epsilon), Throttle) # Fx with epsilon added to Va

    dTdVA = (Fx_eps - Fx) / epsilon # Get partial derivative w/ the formula

    return dTdVA # Return partial of Thrust with respect to Va


def dThrust_dThrottle(Va, Throttle, epsilon=0.01):


    '''
    Function to calculate the numerical partial derivative of propeller thrust to change
    in throttle setting using the actual prop function from complex propeller model (inside the VehicleAerodynamicsModel class)
    '''

    # Same Idea as above different partial this time with respect to throttle

    Fx, Mx = VAM.VehicleAerodynamicsModel.CalculatePropForces(Va, Throttle) # Regular Fx

    Fx_eps, Mx_eps = VAM.VehicleAerodynamicsModel.CalculatePropForces(Va, (Throttle + epsilon)) # Epsilon added to throttle

    dTdT = (Fx_eps - Fx) / epsilon # Get the partial using the formula

    return dTdT # Return partial of Thrust with respect to Throttle.


def CreateTransferFunction(trimState, trimInputs):

    '''
    Function to fill the transfer function parameters used for the successive loop closure from the given trim state and trim inputs. 
    Note that these parameters will be later used to generate actual control loops.
    Vehicle Perturbation models are developed using the trim state and inputs. 
    Models for transfer function parameters and state space implementations are calculated using constants in VehiclePhysicalParameters and the input trim state and trim control inputs.
    Results are returned as a Linearized.transferFunction class.
    '''
    
    # Initialize TF object

    TransFunct = Linearized.transferFunctions()

    # Get all constants needed for the Transfer Function


