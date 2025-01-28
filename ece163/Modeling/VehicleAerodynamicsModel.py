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

        '''Initialization of the internal classes which ar used to track the vehicle aerodynamics and dynamics.'''

        self.Vdynam = VDM.VehicleDynamicsModel() # Assign self all the parameters of the vehicle state from prev Lab 1

        self.Vdynam.state.u = initialSpeed # Velocity in x-dir equals the inital speed this assumes the plane is flying straight and level

        self.Vdynam.state.pd = initialHeight # the initial down position is the intial height

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

        CD_sep = 2  * math.sin(alpha) * math.sin(alpha)  # CD seperated equation 2 sin^2 alpha


        # CL and CD total and CM equations

        CL_tot = ((1 - sigma) * (CL_attach)) + ((sigma) * (CL_sep)) # Given overall CL equation from handout

        CD_tot = ((1 - sigma) * (CD_attach)) + ((sigma) * (CD_sep)) # Given overall CD equation from handout

        CM_tot = VPC.CM0 + (VPC.CMalpha * alpha) # Given overall CM equation from handout


        return CL_tot, CD_tot, CM_tot # Return coefficents of Lift, Drag & Moment



