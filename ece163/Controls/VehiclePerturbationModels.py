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

    Fx , Mx = VAM.VehicleAerodynamicsModel().CalculatePropForces(Va, Throttle) # Regular Fx Mx not used

    nV = Va + epsilon # New Va is Va + epsilon

    Fx_eps, mx_eps = VAM.VehicleAerodynamicsModel().CalculatePropForces(nV, Throttle) # Fx with epsilon added to Va

    dTdVA = (Fx_eps - Fx) / epsilon # Get partial derivative w/ the formula

    return dTdVA # Return partial of Thrust with respect to Va


def dThrust_dThrottle(Va, Throttle, epsilon=0.01):


    '''
    Function to calculate the numerical partial derivative of propeller thrust to change
    in throttle setting using the actual prop function from complex propeller model (inside the VehicleAerodynamicsModel class)
    '''

    # Same Idea as above different partial this time with respect to throttle

    Fx, Mx = VAM.VehicleAerodynamicsModel().CalculatePropForces(Va, Throttle) # Regular Fx

    nT = Throttle + epsilon # New throttle is Throttle pluse epsilon

    Fx_eps, Mx_eps = VAM.VehicleAerodynamicsModel().CalculatePropForces(Va, nT) # Epsilon added to throttle

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

    TransTemp = Linearized.transferFunctions()

    # Get all constants needed for the Transfer Function

    Va = trimState.Va # Get airspeed in the trim

    m = VPC.mass # Get aircraft mass

    alpha_in_trim = trimState.alpha # angle of attack in the trim

    pitch_in_trim = trimState.pitch # pitch angle in the trim

    elev_in_trim = trimInputs.Elevator # elev input in the trim

    throt_in_trim = trimInputs.Throttle # throttle input in the trim

    # Get Phi's

    a_phi_1 = ((-1 / 2) * VPC.rho * (Va ** 2) * VPC.S * VPC.b * VPC.Cpp) * (VPC.b / (2 * Va)) # Eq 5.23 Beard Ch 5

    a_phi_2 = (1 / 2) * VPC.rho * (Va ** 2) * VPC.S * VPC.b * VPC.CpdeltaA # Eq 5.24 Beard Ch 5

    # Get Beta's

    a_beta_1 = -1 * (((VPC.rho * Va * VPC.S) / (2 * m)) * VPC.CYbeta) # Pg 71 of Beard

    a_beta_2 = (((VPC.rho * Va * VPC.S) / (2 * m)) * VPC.CYdeltaR) # Also from Pg 71

    # Get Theta's

    a_theta_1 = (-1) * ((VPC.rho * (Va**2) * VPC.S * VPC.c) / (2*VPC.Jyy)) * VPC.CMq * (VPC.c / (2*Va)) # Pg 73 of Beard

    a_theta_2 = -1 * (((VPC.rho * (Va ** 2) * VPC.c * VPC.S) / (2 * VPC.Jyy)) * VPC.CMalpha) # Also on Pg 73

    a_theta_3 = ((VPC.rho * (Va ** 2) * VPC.c * VPC.S) / (2 * VPC.Jyy)) * VPC.CMdeltaE # Pg 73 as well

    # Get V's from Pg 26 of Beard's supplemental since the textbook equations are aparrently wrong? Also need partials

    dTdT = dThrust_dThrottle(Va, throt_in_trim, 0.01) # Get Partial Deriv w/ resepect to throttle

    dTdVa = dThrust_dVa(Va, throt_in_trim, 0.5) # Partial w/ respect to airspeed in trim

    a_V1 = (((VPC.rho * Va * VPC.S) / m) * (VPC.CD0 + (VPC.CDalpha * alpha_in_trim) + (VPC.CDdeltaE * elev_in_trim))) - ((1 / m) * dTdVa) # Equation for V1 Beard Supl

    a_V2 = (1 / m) * dTdT # V2 equation from Beard Suplemental

    a_V3 = VPC.g0 * (math.cos(pitch_in_trim - alpha_in_trim)) # V3 from Beard Supl

    # Fill the transfer function with calculated parameters

    TransTemp.Va_trim = Va # Fill with trim airspeed

    TransTemp.alpha_trim = alpha_in_trim # fill with trim alpha

    TransTemp.beta_trim = trimState.beta # fill with trim beta

    TransTemp.gamma_trim = (trimState.pitch - trimState.alpha) # fill with trim gamma from Piazza

    TransTemp.theta_trim = pitch_in_trim # fill with trim pitch angle

    TransTemp.phi_trim = trimState.roll # fill with trim roll angle
    
    TransTemp.a_phi1 = a_phi_1 # fill with a phi 1

    TransTemp.a_phi2 = a_phi_2 # fill with a phi 2

    TransTemp.a_beta1 = a_beta_1 # fill with a beta 1

    TransTemp.a_beta2 = a_beta_2 # fill with a beta 2

    TransTemp.a_theta1 = a_theta_1 # fill with a theta 1

    TransTemp.a_theta2 = a_theta_2 # fill with a theta 2

    TransTemp.a_theta3 = a_theta_3 # fill with a theta 3

    TransTemp.a_V1 = a_V1 # fill with aV 1

    TransTemp.a_V2 = a_V2 # fill with aV 2

    TransTemp.a_V3 = a_V3 # fill with aV 3

    return TransTemp # Returned Complete TF