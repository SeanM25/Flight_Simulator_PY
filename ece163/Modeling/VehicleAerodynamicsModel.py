import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel as VDM
from ..Modeling import WindModel
from ..Utilities import MatrixMath as mm
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC


class VehicleAerodynamicsModel:

    def __init__(self, initialSpeed = VPC.InitialSpeed, initialHeight = VPC.InitialDownPosition):

        '''Initialization of the internal classes which are used to track the vehicle aerodynamics and dynamics.'''

        # Check This For Typo?

        self.VState = States.vehicleState() # Assign self all the parameters of the vehicle state from prev Lab 1

        self.VState.u= initialSpeed # Velocity in x-dir equals the inital speed this assumes the plane is flying straight and level

        self.VState.pd = initialHeight # the initial down position is the intial height

        return # Return nothing
    


    
    def CalculateCoeff_alpha(self, alpha):

        # CL & CD Blending Equation From Lecture

        blender_num = 1 + math.exp(-1 * VPC.M * (alpha - VPC.alpha0)) + math.exp(VPC.M * (alpha + VPC.alpha0)) # Numerator of the blending function

        blender_den = (1 + math.exp(-1 * VPC.M * (alpha - VPC.alpha0))) * (1 + math.exp(VPC.M * (alpha + VPC.alpha0)))  # Denominator of the blending function

        sigma = (blender_num / blender_den) # The sigmal value is the blending function numerator over denominator


        # CL (Coefficent of Lift) Equations for Attached (Pre-Stall) and for Seperated (Post-Stall) conditions 
        
        CL_attach = VPC.CL0 + (VPC.CLalpha * alpha) # CL attached equation from lecture, handouts, etc

        CL_sep = 2 * math.sin(alpha) * math.cos(alpha) # CL seperated equation from lab manual


        # CD (Coefficent of Drag) Equations for Attached (Pre-Stall) and for Seperated (Post-Stall) conditions 

        CD_attach_num = (VPC.CL0 + (VPC.CLalpha * alpha)) * (VPC.CL0 + (VPC.CLalpha * alpha)) # Numerator of CD_attached squared

        CD_attach = VPC.CDp + (CD_attach_num / (math.pi * VPC.AR * VPC.e)) # CD attached equation

        CD_sep = (2  * (math.sin(alpha) ** 2))  # CD seperated equation 2 sin^2 alpha


        # CL and CD total and CM equations

        CL_tot = ((1 - sigma) * (CL_attach)) + ((sigma) * (CL_sep)) # Given overall CL equation from handout

        CD_tot = ((1 - sigma) * (CD_attach)) + ((sigma) * (CD_sep)) # Given overall CD equation from handout

        CM_tot = VPC.CM0 + (VPC.CMalpha * alpha) # Given overall CM equation from handout


        return CL_tot, CD_tot, CM_tot # Return coefficents of Lift, Drag & Moment
    



    def CalculatePropForces(self, Va, Throttle):
    
        ''' Function to calculate the propeller forces and torques on the aircraft. 
        Uses the fancy propeller model that parameterizes the torque and thrust coefficients of the propeller using the advance ratio. 
        See ECE163_PropellerCheatSheet.pdf for details. Note: if the propo speed Omega is imaginary, then set it to 100.0 '''
    

    # We need omega for poth the propellor force and the torque so we should first find that

        # Need KT and KV to find omega via the quadratic formula specified in the prop cheat sheet

        KT = (60 / (2 *  math.pi * VPC.KV)) # KT equals the equation directly below (6) in the Prop cheat sheet

        KE = KT # KE = KT according the equation

        # Need Vin as well Vin = Vmax * throttle value according to blurb in the prop cheat sheet

        Vin = VPC.V_max * Throttle

        # Assemble the quadratic equation values a, b, c

        a = ((VPC.rho) * (VPC.D_prop ** 5) * (VPC.C_Q0)) / (4 * math.pi ** 2) # Given a

        b = (((VPC.rho) * (VPC.D_prop ** 4) * (Va) * (VPC.C_Q1)) / (2 * math.pi)) + ((KT * KE) / (VPC.R_motor)) # Given b

        c = ((VPC.rho) * (VPC.D_prop ** 3) * (Va ** 2) * (VPC.C_Q2)) - (KT * (Vin/VPC.R_motor)) + (KT * VPC.i0) # Given c

        # Check for Imaginary omega

        if ((b ** 2) < (4 * a * c)): # For the quadratic formula if b^2 < 4ac then omega must be an imaginary num that is omega = (- b +/- j / 2a )

            omega = 100.0 # If imaginary make omega 100

        else:

            omega = ((-1 * b) + math.sqrt((b ** 2) - (4 * a * c)) / (2 * a)) # Otherwise calculate omega using the quadratic formula normally

        # Get J to find CT & CQ

        J = ((2 * math.pi * Va) / (omega * VPC.D_prop))

        # Get CQ & CT for final calculation

        CT = VPC.C_T0 + (VPC.C_T1 * J) + (VPC.C_T2 * (J ** 2)) # Equation (3)

        CQ = VPC.C_Q0 + (VPC.C_Q1 * J) + (VPC.C_Q2 * (J ** 2)) # Equation (4)

        
        Fx = (((VPC.rho) * (omega ** 2) * (VPC.D_prop ** 4) * (CT)) / (4 * (math.pi ** 2))) # Equation (1) Force of Prop

        Mx = (((VPC.rho) * (omega ** 2) * (VPC.D_prop ** 5) * (CQ)) / (4 * (math.pi ** 2))) # Equation (2) Moment of prop

        return Fx, Mx # Return Prop Force and Moment



    def setVehicleState(self, state):

        self.VState = state # Set Vehicle State to current state

        return # Return nothing










