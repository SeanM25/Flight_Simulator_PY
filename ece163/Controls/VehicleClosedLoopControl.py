import math
import sys
import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule
import ece163.Controls.VehicleEstimator as VehicleEstimator
import ece163.Sensors.SensorsModel as SensorsModel


class PDControl:

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
    

    
    def setPDGains(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):

        '''Function to set the gains for the PD control block (including the trim output and the limits)'''

    # Basically this function is kinda like init but rather than initializing we are changing values on the fly

        # Parameters:
            # kp: proportional gain
            # kd : derivative gain
            # trim : trim output
            # lowLimit : lower limit of accumulator saturation
            # highLimit : upper limit of accumulator saturation 

        self.kp = kp # kp is new Kp

        self.kd = kd # kd is new kd

        self.trim = trim # trim is new trim

        self.lowLimit = lowLimit # lowLimit is new lowLimit

        self.highLimit = highLimit # highLimit is new highLimit

        return # return nothing
    
class PIControl:

    def __init__(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):

        '''Functions which implement the PI control with saturation where the integrator has both a reset and an anti-windup such that when output saturates, 
       the integration is undone and the output forced the output to the limit. The output is: u = u_ref + Kp * error + Ki * integral{error} limited between lowLimit and highLimit.''' 
        
    # Parameters:
            # kp: proportional gain
            # kd : derivative gain
            # trim : trim output
            # lowLimit : lower limit of accumulator saturation
            # highLimit : upper limit of accumulator saturation 

            # To perform integration we use the trapezoidal method outlined in lecture accumulator = 1/2 (error + prev error) dT

            # As such, we must define a accumulator member and a previous error member as well

        self.dT = dT # Timestep set to 0.01

        self.kp = kp # proportional gain set to desired value

        self.ki = ki # integral gain set to desired valuue

        self.trim = trim # trim output set to given value

        self.lowLimit = lowLimit # lower limit of saturation specified

        self.highLimit = highLimit # upper limit of saturation specified

        self.accumulator = 0.0 # Initial accumulator state for integration is set to 0

        self.pastError = 0.0 # Initial past error state is 0
        
        return # return nothing
    
    def Update(self, command=0.0, current=0.0):

        '''Calculates the output of the PI loop given the gains and limits from instantiation, and using the command and current or actual inputs.
          Output is limited to between lowLimit and highLimit from instantiation. 
          Integration for the integral state is done using trapezoidal integration, and anti-windup is implemented such that if the output is out of limits, the integral state is not updated (no additional error accumulation).'''

        # Get the error

        error = command - current # input error equals reference command minus system output

        # Do the accumulation that is trapezoidal  integration

        self.accumulator += (1 / 2) * (error + self.pastError) * (self.dT)  # Accumulator trapez equation from lecture

        # Get u 

        u = self.trim + (self.kp * error) + (self.ki * self.accumulator) # Given PI control equation

        # Check saturation bounds

        if (u > self.highLimit): # If we're at the upper limit of saturation

            u = self.highLimit # set u to the upper limit

            self.accumulator -= (1 / 2) * (error + self.pastError) * (self.dT) # Decrement the accumulator as specified in lecture

        elif(u < self.lowLimit): # If we're at the lower limit of saturation

            u = self.lowLimit # set u to the lower limit

            self.accumulator -= (1 / 2) * (error + self.pastError) * (self.dT) # Decrement the accumulator as specified in lecture

        # Change past Error state

        self.pastError = error # current error becomes the past error state

        return u # # Return PI control output
    
    def resetIntegrator(self):

        # Function to reset the integration state to zero, used when switching modes or otherwise resetting the integral state

        # This function resets the acumulator and past error state back to 0

        self.accumulator = 0.0 # accumulator reset to 0

        self.pastError = 0.0 # reset past error state to 0

        return # return nothing
    
    def setPIGains(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):

        # Function to set the gains for the PI control block (including the trim output and the limits)

        self.dT = dT # set PI time step

        self.kp = kp # set kp for PI

        self.ki = ki # set ki fpr PI

        self.trim = trim # set trim output for PI

        self.lowLimit = lowLimit # set lower saturation limit for PI

        self.highLimit = highLimit # set upper saturation limit for PI

        return # return nothing
    

class PIDControl:

    # These functions are pretty much a copy paste of PI control with the primary differnece being the derivative term

    '''Functions which implement the PID control with saturation where the integrator has both a reset and an anti-windup such that when output saturates, the integration is undone and the output forced the output to the limit. 
    Function assumes that physical derivative is available (e.g.: roll and p), not a numerically derived one. The output is: u = u_ref + Kp * error - Kd * dot{error} + Ki * integral{error} limited between lowLimit and highLimit.'''

    def __init__(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):

         # Parameters:
            # kp: proportional gain
            # kd : derivative gain
            # trim : trim output
            # lowLimit : lower limit of accumulator saturation
            # highLimit : upper limit of accumulator saturation 

            # To perform integration we use the trapezoidal method outlined in lecture accumulator = 1/2 (error + prev error) dT

            # As such, we must define a accumulator member and a previous error member as well

        self.dT = dT # Timestep set to 0.01

        self.kp = kp # proportional gain set to desired value

        self.kd = kd # derivative gain set to desired value

        self.ki = ki # integral gain set to desired valuue

        self.trim = trim # trim output set to given value

        self.lowLimit = lowLimit # lower limit of saturation specified

        self.highLimit = highLimit # upper limit of saturation specified

        self.accumulator = 0.0 # Initial accumulator state for integration is set to 0

        self.pastError = 0.0 # Initial past error state is 0
        
        return # return nothing
    
    def Update(self, command=0.0, current=0.0, derivative=0.0):

         # Get the error

        error = command - current # input error equals reference command minus system output

        # Do the accumulation that is trapezoidal  integration

        self.accumulator += (1 / 2) * (error + self.pastError) * (self.dT)  # Accumulator trapez equation from lecture

        # Get u 

        u = self.trim + (self.kp * error) + (self.ki * self.accumulator) - (self.kd * derivative) # Given PI control equation

        # Check saturation bounds

        if (u > self.highLimit): # If we're at the upper limit of saturation

            u = self.highLimit # set u to the upper limit

            self.accumulator -= (1 / 2) * (error + self.pastError) * (self.dT) # Decrement the accumulator as specified in lecture

        elif(u < self.lowLimit): # If we're at the lower limit of saturation

            u = self.lowLimit # set u to the lower limit

            self.accumulator -= (1 / 2) * (error + self.pastError) * (self.dT) # Decrement the accumulator as specified in lecture

        # Change past Error state

        self.pastError = error # current error becomes the past error state

        return u # # Return PI control output
    
    
    



    






