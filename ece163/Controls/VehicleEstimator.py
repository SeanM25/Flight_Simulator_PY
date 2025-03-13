import math
from ..Containers import Controls
from ..Containers import Sensors
from ..Containers import States
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleDynamicsModel as VDM
from ..Sensors import SensorsModel
from ..Utilities import MatrixMath as MM


class LowPassFilter:

    def __init__(self, dT = 0.01, cutoff = 1):

        self.dT = dT # Set timestep of the LPF

        self.cutoffDef = 1 # Default cutooff of 1 Hz

        self.cutoff = cutoff # actual cutoff freq Hz

        self.yk = 0.0 # Initial output of LPF

        self.yk_prev = 0.0 # Initial prev state of LPF

        return # return nothing
    

    def reset(self):

        # reset internal storgage variables that is yk and yk_prev

        self.yk = 0.0 # Reset output to zero

        self.yk_prev = 0.0 # Reset prev yk to zero

        return # return nothing
    

    

