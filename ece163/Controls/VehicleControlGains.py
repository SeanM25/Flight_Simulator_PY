import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations



class PDControl(kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):

    '''Functions which implement the PD control with saturation where the derivative is available as a separate input to the function. The output is: u = u_ref + Kp * error - Kd * dot{error} limited between lowLimit and highLimit.'''

    def __init__(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        
    # Here we set the intial conditions for the Proportional Derivative controller to the given values which default to 0

    # Paramaterss:

        # kp: proportional gain
        # kd : derivative gain
        # trim : trim output
        # lowLimit : lower limit of accumulator saturation
        # highLimit : upper limit of accumulator saturation

        self.kp = kp # set proprtional gain to given proprtional gain

        self.kd = kd # set derivative gain to given deriv gain

        self.trim = trim # set trim output to given trim output (added the the loop computed output)

        self.lowLimit = lowLimit # set lower limit saturation

        self.highLimit = highLimit # set upper limit of saturation

        return # Return nothing
    
    def Update(self, command=0.0, current=0.0, derivative=0.0):

        '''Calculates the output of the PD loop given the gains and limits from instantiation, and using the command, actual, and derivative inputs. Output is limited to between lowLimit and highLimit from instantiation.'''

    # Parameters:

        # command: Reference input command

        # current: Actual output or sensor

        # derivative: output or sensor derivative

        error = command - current # get error which is the input command minus the output actual

        u = self.trim + (self.kp * error) - (self.kd * derivative) # Given equation with trim added in as u_ref

        # Check for saturation:

        if (u > self.highLimit): # If u is greater than the upper saturation limit then set it to the upper limit

            u = self.highLimit

        elif (u < self.lowLimit): # If u is smaller than the lower saturation limit then set it to the lower limit

            u = self.lowLimit


        return u # Return PD control output
    

    

