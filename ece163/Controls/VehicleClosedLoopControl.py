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

        u = self.trim + (self.kp * error) + (self.ki * self.accumulator) - (self.kd * derivative) # Given PID control equation

        # Check saturation bounds

        if (u > self.highLimit): # If we're at the upper limit of saturation

            u = self.highLimit # set u to the upper limit

            self.accumulator -= (1 / 2) * (error + self.pastError) * (self.dT) # Decrement the accumulator as specified in lecture

        elif(u < self.lowLimit): # If we're at the lower limit of saturation

            u = self.lowLimit # set u to the lower limit

            self.accumulator -= (1 / 2) * (error + self.pastError) * (self.dT) # Decrement the accumulator as specified in lecture

        # Change past Error state

        self.pastError = error # current error becomes the past error state

        return u # # Return PID control output
    
    def resetIntergrator(self):

        # Same for PID as it was with PI

        # This function resets the acumulator and past error state back to 0

        self.accumulator = 0.0 # accumulator reset to 0

        self.pastError = 0.0 # reset past error state to 0

        return # return nothing
    
    def setPIDGains(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):

        # Function to set the gains for the PI control block (including the trim output and the limits)

        self.dT = dT # set PID time step

        self.kp = kp # set kp for PID

        self.kd = kd # set kd for PID

        self.ki = ki # set ki fpr PID

        self.trim = trim # set trim output for PID

        self.lowLimit = lowLimit # set lower saturation limit for PID

        self.highLimit = highLimit # set upper saturation limit for PID

        return # return nothing
    
    
class VehicleClosedLoopControl:

    def __init__(self, dT=0.01, rudderControlSource='SIDESLIP', useSensors=False, useEstimator=False):

        # We can ignore everything besides dT until Lab 5 & 6

        # Lab 5 Modifications:

        self.useSensors = useSensors # Assign use sensors Boolean

        if(self.useSensors): # If the use sensors boolean is true

            self.sensorsModel = SensorsModel.SensorsModel(aeroModel = VehicleAerodynamicsModule.VehicleAerodynamicsModel()) # Assign sensors model param
            


        self.VAM = VehicleAerodynamicsModule.VehicleAerodynamicsModel() # Give an VAM instance for the class

        self.controlGains = Controls.controlGains() # Give a control gains instance to keep track of Kp, Ki Kd

        # Two controlInputs one for trim inputs and the other for Vehicle Aerodynamics module

        self.trimInputs = Inputs.controlInputs() # Gets throttle, aileron, elevator, rudder parameters for the trim state

        self.VehicleControlSurfaces = Inputs.controlInputs() # Gets throttle, aileron, elevator, rudder parameters for the vehicle control surfaces that are fed into VAM

        # Store the given dT

        self.dT = dT # Store the given dT value as well

        # Intialize controller mode for State Machine

        self.climbState = Controls.AltitudeStates.HOLDING # Intializes the mode of the controller to Holding other possible modes are Climbing & Decending

        # Create the 7 feedback contollers

        # PI Controllers

        self.rollFromCourse = PIControl()

        self.rudderFromSideslip = PIControl()

        self.throttleFromAirspeed = PIControl()

        self.pitchFromAltitude = PIControl()

        self.pitchFromAirspeed = PIControl()

        # PD Controller

        self.elevatorFromPitch = PDControl()

        # PID Controller

        self.aileronFromRoll = PIDControl()

        return # return nothing
    

    def getControlGains(self):

        # wrapper function to extract control gains from class

        return self.controlGains # return control gains
    
    def getVehicleState(self):

        # Wrapper function to extract vehicle state from the class.

        return self.VAM.VDynamics.state # retun the Vehicle state
    
    def setVehicleState(self, state):

        # wrapper function to set that is inject the vehicle state

        self.VAM.VDynamics.state = state # Set new vehicle stae

        return # return nothing
    
    def getTrimInputs(self):

        # Wrapper function that gets the current trim inputs

        return self.trimInputs # return present trim inputs
    
    def setTrimInputs(self, trimInputs = Inputs.controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)):

        # Wrapper function that injects the current trim inputs

        self.trimInputs = trimInputs # inject new trim inputs

        return  # return nothing
    
    def getVehicleAerodynamicsModel(self):

        # Gets current VAM & attributes

        return self.VAM # return vehicle aerodynamics modes
    
    def getVehicleControlSurfaces(self):

        # Gets current Vehicle Control Surfaces and attributes

        return self.VehicleControlSurfaces # Get current Control Surfaces


    def setControlGains(self, controlGains=Controls.controlGains()):

        # Function calls the appropriate set gains function for each controller : PD, PI, or PID

        # Some parameters must be converted from degrees to rads using math.radians()

        self.controlGains = controlGains # update control gains parameter from function argument

        trimInputs = self.trimInputs # Get current trim inputs

        dT = self.dT # Get current timestep dT
        
        # Set PD Controller Gain (Elevator from Pitch)

        kp_pitch_EFP = self.controlGains.kp_pitch # Get prop gain for elevator from pitch

        kd_pitch_EFP = self.controlGains.kd_pitch # Get deriv gain for elevator from pitch
        
        elev_minLimit = VPC.minControls.Elevator # Get the lower limit for the elevator from VPC min/max controls

        elev_maxLimit = VPC.maxControls.Elevator # Get the upper limit for the elevator from VPC min/max controls

        self.elevatorFromPitch.setPDGains(kp_pitch_EFP, kd_pitch_EFP, trimInputs.Elevator, elev_minLimit, elev_maxLimit) # Set gains for elevatorFromPitch Controller

        # Set PID Controller Gain (Aileron from roll)

        kp_AFR = self.controlGains.kp_roll # Aileron from roll prop gain

        kd_AFR = self.controlGains.kd_roll # Aileron from roll deriv gain

        ki_AFR = self.controlGains.ki_roll # Aileron from roll int gain

        aileron_MIN = VPC.minControls.Aileron # min aileron limit

        aileron_MAX = VPC.maxControls.Aileron # max aileron limit

        self.aileronFromRoll.setPIDGains(dT, kp_AFR, kd_AFR, ki_AFR, trimInputs.Aileron, aileron_MIN, aileron_MAX) # Update Aileron From Roll gains

        # Set PI Controller Gains for rollFromCourse, rudderFromSideslip, throttleFromAirspeed, pitchFromAltitude, pitchFromAirspeed

            # Roll From Course

        kp_RFC = self.controlGains.kp_course # Course gain used to get the roll

        ki_RFC = self.controlGains.ki_course # Course integral gain used to get the roll

        RFC_lowerLimit = -1 * math.radians(VPC.bankAngleLimit) # convert the lower limit of integration for Roll to Course to rads from degrees this is the bank angle limit

        RFC_upperLimit = math.radians(VPC.bankAngleLimit) # convert the upper limit of integration for Roll to Course to rads from degrees this is the bank angle limit

        self.rollFromCourse.setPIGains(dT, kp_RFC, ki_RFC, 0.0, RFC_lowerLimit, RFC_upperLimit) # Upfate Roll From Course gains

            # Rudder From Sideslip

        kp_RSS = self.controlGains.kp_sideslip # Sideslip gain used in Rudder from Sideslip

        ki_RSS = self.controlGains.ki_sideslip # Sideslip integral gain used in Rudder from Sideslip

        RSS_lowLimit = VPC.minControls.Rudder # Rudder lower limit for integration

        RSS_highLimit = VPC.maxControls.Rudder # Rudder upper limit for integration

        self.rudderFromSideslip.setPIGains(dT, kp_RSS, ki_RSS, trimInputs.Rudder, RSS_lowLimit, RSS_highLimit) # Update rudder from sideslip gains

            # Throttle From Airspeed 

        kp_TFS = self.controlGains.kp_SpeedfromThrottle # Proportion gain Throttle from airspeed

        ki_TFS = self.controlGains.ki_SpeedfromThrottle # Integral gain throttle from airspeed

        TFS_lowLimit = VPC.minControls.Throttle # Throttle lower limit

        TFS_highLimit = VPC.maxControls.Throttle # throttle upper limit

        self.throttleFromAirspeed.setPIGains(dT, kp_TFS, ki_TFS, trimInputs.Throttle, TFS_lowLimit, TFS_highLimit) # update throttle from airspeed gains

            # Pitch from Altitutde    

        kp_PFA = self.controlGains.kp_altitude # proprtional gain pitch from altitude

        ki_PFA = self.controlGains.ki_altitude # integral gain pitch from altitude

        PFA_lowLimit = -1 * math.radians(VPC.pitchAngleLimit) # lower limit of integration for pitch

        PFA_highLimit = math.radians(VPC.pitchAngleLimit) # upper limit of integration for pitch

        self.pitchFromAltitude.setPIGains(dT, kp_PFA, ki_PFA, 0.0, PFA_lowLimit, PFA_highLimit) # update pitch from altitude gains

        # Pitch from Airspeed

        kp_pitchAir = self.controlGains.kp_SpeedfromElevator # pitch from airspeed gain

        ki_pitchAir = self.controlGains.ki_SpeedfromElevator # pitch from airspeed integral gain

        pitch_AirlowLim = -1 * math.radians(VPC.pitchAngleLimit) # lower limit of integration for pitch

        pitch_AirhighLim = math.radians(VPC.pitchAngleLimit) # upper limit of integration for pitch

        self.pitchFromAirspeed.setPIGains(dT, kp_pitchAir, ki_pitchAir, 0.0, pitch_AirlowLim, pitch_AirhighLim) # update pitch from airspeed gains

        return # return nothing
    

    def getSensorsModel(self):

        # Simple geter that returns the snesors model if we're using the sensors

        if(self.useSensors):

            sensorModel = self.sensorsModel

            return sensorModel

        return # Otherwise return nothing
    

    def reset(self):

        # Resets VAM and all the controllers except elevatorFromPitch since there is no integral term

        if(self.useSensors): # if the use sensors boolean is true

            self.sensorsModel.reset() # Reset the sensor model



        self.VAM = self.VAM.reset() # Reset VAM

        self.rollFromCourse = self.rollFromCourse.resetIntegrator() # reset roll from course

        self.rudderFromSideslip = self.rudderFromSideslip.resetIntegrator() # reset rudder from sideslip

        self.throttleFromAirspeed = self.throttleFromAirspeed.resetIntegrator() # reset throttle from airspeed

        self.pitchFromAltitude = self.pitchFromAltitude.resetIntegrator() # reset pitch from alitutde

        self.pitchFromAirspeed = self.pitchFromAirspeed.resetIntegrator() # reset pitch from airspeed

        self.aileronFromRoll = self.aileronFromRoll.resetIntergrator() # reset aileron from roll

        return # return nothing


    def UpdateControlCommands(self, referenceCommands, state):

        # Implements the Altitude Hold State Machine outlined in Lab manual and lecture

        # All logic comes from Lecture Diagrams / State Machine in Lab Manual

        curAlt = -state.pd # Gets current altitutde

        controlSurfaceOutputs = Inputs.controlInputs() # Creates a control inputs instance to fill and return

        upper_threshold = referenceCommands.commandedAltitude + VPC.altitudeHoldZone # Upper threshold given in handout

        lower_threshold = referenceCommands.commandedAltitude - VPC.altitudeHoldZone # Lower threshold given in handout

        # Get and check course error

        courseError = referenceCommands.commandedCourse - state.chi # Get course error as defined in lecture

        if(courseError >= math.pi): # If greater than pi incrment chi by 2pi

            state.chi += 2 * math.pi
        
        elif(courseError <= -math.pi): # If ledd than -pi decrement chi by 2pi

            state.chi -= 2 * math.pi

        
        # State Machine Start

        if(curAlt > upper_threshold): # We need to be Decending

            if(self.climbState != Controls.AltitudeStates.DESCENDING): # If not in decending mode

                self.climbState = Controls.AltitudeStates.DESCENDING # Set mode to decending

                self.pitchFromAirspeed.resetIntegrator() # Reset Airspeed Integrator

                # Decending State

            pitchCOM = self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va) # Get current pitch command

            throttleCOM = VPC.minControls.Throttle # Set throttle to maximum

        elif(curAlt < lower_threshold): # We need to be Climbing

            if(self.climbState != Controls.AltitudeStates.CLIMBING): # If not climbing

                self.climbState = Controls.AltitudeStates.CLIMBING # Set to climbing

                self.pitchFromAirspeed.resetIntegrator() # Reset Airspeed Integrator

                # Climbing State

            pitchCOM = self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va) # Get pitch from airspeed

            throttleCOM = VPC.maxControls.Throttle # Throttle to maximim

        else: # If not Cimbing or Decending We're in Holding

            if(self.climbState != Controls.AltitudeStates.HOLDING): # If not in Holding Mode

                self.climbState = Controls.AltitudeStates.HOLDING # Set to Holding

                self.pitchFromAirspeed.resetIntegrator() # Reset Integrator for Airspeed

                self.pitchFromAltitude.resetIntegrator() # Reset Integrator for Altitude

            pitchCOM = self.pitchFromAltitude.Update(referenceCommands.commandedAltitude, curAlt) # Get Pitch from Altitude

            throttleCOM = self.throttleFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va) # Get throttle from Airspeed

        # If we're not in one of climbing states we update everything as outlined in lecture

        referenceCommands.commandedRoll = self.rollFromCourse.Update(referenceCommands.commandedCourse, state.chi) # Update roll command

        controlSurfaceOutputs.Aileron = self.aileronFromRoll.Update(self.rollFromCourse.Update(referenceCommands.commandedCourse, state.chi), state.roll, state.p) # Update aileron command

        controlSurfaceOutputs.Rudder = self.rudderFromSideslip.Update(0.0, state.beta) # update rudder command

        controlSurfaceOutputs.Throttle = throttleCOM # Update throttle command as determined by the climb states

        referenceCommands.commandedPitch = pitchCOM # Update pitch command as determined by the climb states

        controlSurfaceOutputs.Elevator = self.elevatorFromPitch.Update(pitchCOM, state.pitch, state.q) # Update throttle commands

        self.VehicleControlSurfaces = controlSurfaceOutputs # Set Vehicle Control Surfaces attribute for class

        return controlSurfaceOutputs # Return control surfaces

         
    def update(self, referenceCommands=Controls.referenceCommands):

        if(self.useSensors): # If we're using sensors

            self.sensorsModel.update() # update the senors

        state = self.getVehicleState() # Get current state

        ControlCommands = self.UpdateControlCommands(referenceCommands, state) # Call the autopilot and get the Control Surfaces commands

        self.VAM.Update(ControlCommands) # Update VAM with said commands

        return # return nothing



        

        






            


        

        



        


    

    



    






