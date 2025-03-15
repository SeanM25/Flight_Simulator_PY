import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Utilities import MatrixMath as mm
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel
from ..Containers import States



class GaussMarkov:

    def __init__(self, dT = 0.01, tau = 1.0e6, eta = 0.0):

        # Sets Inital Parameters to Compute the Gauss Markov Model


        self.tau = tau # Get tau parameter

        self.eta = eta # Get eta

        self.dT = dT # get time step

        self.prev_V  = 0.0 # Prev GM state is intitialized to 0

        return # Return nothing
    

    def reset(self):

        # Resets All Gauss Markov Parameters to IC's

        self.dT = 0.01 # reset time step

        self.tau = 1.0e6 # Reset tau

        self.eta = 0.0 # Reset eta

        self.prev_V = 0.0 # Reset prev GM state

        return # return nothing


    def update(self, vnoise = None):

        # Updates and Computes the Gauss Markov function with the option to drive GM with known value vnoise

        tau = self.tau # Get current tau

        dT = self.dT # get current time step

        if (self.eta == None): # If 0 then random.gauss(0,0) which is just 0

            w = 0.0 # w is 0 this is the random number from GM
        else:

            w = random.gauss(0, self.eta) # Otherwise random.gauss(0, eta)

        
        if(vnoise == None): # If were not driving with a known value 

            V = (math.exp(-(dT / tau)) * self.prev_V) + w # Use random gauss

        else:

            V = (math.exp(-(dT / tau)) * self.prev_V) + vnoise # otherwise use Vnoise

        self.prev_V = V # set previous GM state to current GM state

        return V # Return GM


class GaussMarkovXYZ:

    def __init__(self, dT=VPC.dT, tauX=1e6, etaX=0.0, tauY=None, etaY=None, tauZ=None, etaZ=None):

        # Function creates three Gauss Markov models one for each axis X Y or Z

        # If the Y or Z Parameters are none we default to their X values

        # Due to the random nature of GM even with default values each process will develop differently
        

        self.dT_XYZ = dT # Assign timestep for GM XYZ

        self.tauX = tauX # Assign tauX

        self.etaX = etaX # Assign etaX

        # Check Y Conds

        if(tauY == None): # If Tau Y is none

            self.tauY = tauX # set Tau Y to Tau X
        else:

            self.tauY = tauY # Other wise set Tau Y = Tau Y

        if(etaY == None): # If etaY is none

            self.etaY = etaX # Set equal to etaX
        else:

            self.etaY = etaY # Otherwise set equal to etaY

        # Check Z Conds

        if(tauZ == None): # If Tau Z is none

            self.tauZ = tauX # set Tau Z to Tau X
        else:

            self.tauZ = tauZ # Other wise set Tau Z = Tau Y

        if(etaZ == None): # If etaZ is none

            self.etaZ = etaX # Set equal to etaZ
        else:

            self.etaZ = etaZ # Otherwise set equal to etaZ


        # Create GM Objects for each axis


        self.GM_XYZ_X = GaussMarkov(self.dT_XYZ, self.tauX, self.etaX) # Create X axis Gauss Markov Object

        self.GM_XYZ_Y = GaussMarkov(self.dT_XYZ, self.tauY, self.etaY) # Create Y axis Gauss Markov Object

        self.GM_XYZ_Z = GaussMarkov(self.dT_XYZ, self.tauY, self.etaZ) # Create Z axis Gauss Markov Object

        return # Return nothing
    
    def reset(self):

        # Resets GM XYZ models

        self.GM_XYZ_X = self.GM_XYZ_X.reset() # reset GM_XYZ X

        self.GM_XYZ_Y = self.GM_XYZ_Y.reset() # reset GM_XYZ Y

        self.GM_XYZ_Z = self.GM_XYZ_Z.reset() # reset GM_XYZ Z

        return # return nothing
    
    def update(self, vXnoise=None, vYnoise=None, vZnoise=None):

        # Updates each of the GM XYZ object indvidually by calling the Gen Gauss Markov

        # Updates with optional parameter Vnoise


        Vx = self.GM_XYZ_X.update(vXnoise) # Update X axis

        Vy = self.GM_XYZ_Y.update(vYnoise) # Update Y axis

        Vz = self.GM_XYZ_Z.update(vZnoise) # Update Z axis

        return Vx, Vy, Vz # Return new noise values for each axis
    

class SensorsModel:

    def __init__(self, aeroModel = VehicleAerodynamicsModel.VehicleAerodynamicsModel(), taugyro = VSC.gyro_tau, etagyro = VSC.gyro_eta, tauGPS = VSC.GPS_tau, etaGPSHorizontal = VSC.GPS_etaHorizontal, etaGPSVertical = VSC.GPS_etaVertical, gpsUpdateHz = VSC.GPS_rate):

        self.VAM = aeroModel # Keep an instance of Vehicle Aerodynamics Module.

        # Create four different sensor containers for True readings, Biases, Sigmas, and Noise

        self.sensorsTrue = Sensors.vehicleSensors() # Container for True readings

        self.sensorsBiases = self.initializeBiases() # Container for Bias readings call Bias function to intialize

        self.sensorsSigmas = self.initializeSigmas() # Container for Sigma readings call Sigma function to intialize

        self.sensorsNoisy = Sensors.vehicleSensors() # Container for Noisy readings

        # Get time step dT

        self.dT = self.VAM.VDynamics.dT # Get dT

        # Intialize Gauss Markov for both Gyro & GPS. Use GM XYZ since they both measure in 3-D

        self.Gyro_GM_XYZ = GaussMarkovXYZ(self.dT, taugyro, etagyro) # Create a GM XYZ for the GPS using the passed in tau and eta for the gyro

        self.gps_dT = (1 / gpsUpdateHz) # Period or timestep of the GPS

        self.GPS_GM_XYZ = GaussMarkovXYZ(self.gps_dT, tauGPS, etaGPSHorizontal, tauGPS, etaGPSHorizontal, tauGPS, etaGPSVertical)

        # Create a GM XYZ for the GPS using tau gps for tau and the appropriate etas for X,Y, and Z
        
        self.updateTicks = 0 # Intialize tick counter to zero. This tells us when its time to update the GPS

        self.gpsTickUpdate = (self.gps_dT / self.dT) # Get number of dT's that fit in a GPS update period

        return # Return nothing
    

    def initializeSigmas(self, gyroSigma = VSC.gyro_sigma, accelSigma = VSC.accel_sigma, magSigma = VSC.mag_sigma, baroSigma = VSC.baro_sigma, pitotSigma = VSC.pitot_sigma, gpsSigmaHorizontal = VSC.GPS_sigmaHorizontal, gpsSigmaVertical = VSC.GPS_sigmaVertical, gpsSigmaSOG = VSC.GPS_sigmaSOG, gpsSigmaCOG = VSC.GPS_sigmaCOG):

        # Gets and returns appropriate sigmas for each variable within an instance of vehicleSensors

        
        sensorSigmas = Sensors.vehicleSensors() # Create vehicle sensors instance to fill

        # Assign Gyro sigmas

        sensorSigmas.gyro_x = gyroSigma # Sigma for gyro in x

        sensorSigmas.gyro_y = gyroSigma # Sigma for gyro in y

        sensorSigmas.gyro_z = gyroSigma # Sigma for gyro in z

        # Assign accelerometer sigmas

        sensorSigmas.accel_x = accelSigma # Sigma for Accelerometer in X

        sensorSigmas.accel_y = accelSigma # Sigma for Accelerometer in Y

        sensorSigmas.accel_z = accelSigma # Sigma for Accelerometer in Z

        # Assign magnetometer sigmas

        sensorSigmas.mag_x = magSigma # Sigma for Magnetometer in X

        sensorSigmas.mag_y = magSigma # Sigma for Magnetometer in Y

        sensorSigmas.mag_z = magSigma # Sigma for Magnetometer in Z

        # Assign Baro and Pitot sigma

        sensorSigmas.baro = baroSigma # Sigma for Baro

        sensorSigmas.pitot = pitotSigma # Sigma for Pitot

        # Assign GPS Sigmas

        sensorSigmas.gps_cog = gpsSigmaCOG # Assign GPS Course over Ground Sigma

        sensorSigmas.gps_sog = gpsSigmaSOG # Assign GPS Speed over Ground Sigma

        sensorSigmas.gps_n = gpsSigmaHorizontal # GPS N gets Sigma for white noise in Horizontal

        sensorSigmas.gps_e = gpsSigmaHorizontal # GPS E gets Sigma for white noise in Horizontal

        sensorSigmas.gps_cog = gpsSigmaVertical # GPS DWN gets Sigma for white noise in Horizontal

        return sensorSigmas # Return filled sensorSigmas container
    
    def initializeBiases(self, gyroBias = VSC.gyro_bias, accelBias = VSC.accel_bias, magBias = VSC.mag_bias, baroBias = VSC.baro_bias, pitotBias = VSC.pitot_bias):

        # Creates and fills a sensor bias container
        # Gets random number from random.uniform(-1, 1) according to documentation says -gyro_bias gyro_bias in lab doc
        # Very similar to sigma function

        sensorBiases = Sensors.vehicleSensors() # Create empty sensorBiases container to fill

        limit = 1 # uniform dist limit

        # We select a random number from the uniform dist then multiply by the passed in biases

        # Get Gyro Biases for X, Y, and Z

        sensorBiases.gyro_x = random.uniform(-limit, limit) * gyroBias

        sensorBiases.gyro_y = random.uniform(-limit, limit) * gyroBias

        sensorBiases.gyro_z = random.uniform(-limit, limit) * gyroBias

        # Get Accelerometer Biases for X, Y, and Z

        sensorBiases.accel_x = random.uniform(-limit, limit) * accelBias

        sensorBiases.accel_y = random.uniform(-limit, limit) * accelBias

        sensorBiases.accel_z = random.uniform(-limit, limit) * accelBias

        # Get Magnetometer biases for X, Y, and Z

        sensorBiases.mag_x = random.uniform(-limit, limit) * magBias

        sensorBiases.mag_y = random.uniform(-limit, limit) * magBias

        sensorBiases.mag_z = random.uniform(-limit, limit) * magBias

        # Get Baro and Pitot biases

        sensorBiases.baro = random.uniform(-limit, limit) * baroBias

        sensorBiases.pitot = random.uniform(-limit, limit) * pitotBias

        # Get GPS Biases

        # Note: GPS is unbiased so set all biases to 0.0

        sensorBiases.gps_alt = 0.0

        sensorBiases.gps_cog = 0.0

        sensorBiases.gps_e = 0.0

        sensorBiases.gps_sog = 0.0

        sensorBiases.gps_n = 0.0


        return sensorBiases # return all biases
    
    def getSensorsTrue(self):
        
        # Wrapper that returns present sensors true

        return self.sensorsTrue # return sensorsTrue
    
    def setSensorsTrue(self, sensorsTrue = Sensors.vehicleSensors()):

        # Simple setter to set a new sensors true state

        self.sensorsTrue = sensorsTrue # set new sensors true state

        return # return nothing
    
    def getSensorsNoisy(self):

        # Wrapper for sensors nosiy

        return self.sensorsNoisy # return sensorsNoisy
    
    def setSensorsNoisy(self, sensorsNoisy = Sensors.vehicleSensors()):

        # Simple setter to set a new sensorsNoisy state

        self.sensorsNoisy = sensorsNoisy # Set new sensorsNoisy state

        return # return nothing
    
    def reset(self):

        # reset all sensors

        self.sensorsTrue = Sensors.vehicleSensors() # reset sensors true

        self.sensorsNoisy = Sensors.vehicleSensors() # reset sensors noisy

        # Recalculate biases and sigmas

        self.sensorsBiases = self.initializeBiases() # Recalculate biases

        self.sensorsSigmas = self.initializeSigmas() # Recalculate sigmas

        # Reset Gauss Markov for GPS and Gyro

        self.GPS_GM_XYZ.reset() # Reset GPS GM

        self.Gyro_GM_XYZ.reset()  # Reset Gyro GM

        return # return nothing
    
    def updateGyrosTrue(self, state):

        # Simple one liner that just returns p, q, and r which the gyro measures

        return state.p, state.q, state.r # Return p q r
    
    def updateMagsTrue(self, state):

        # This function gets and returns the body frame magnetic field. 

        # As such we must rotate the magnetic field from VSC into the body frame

        R = state.R # Get rotation matrix inertial to body

        mag_field_inertial = VSC.magfield # Get magnetic field in inertial frame NED

        mag_field_body = mm.multiply(R, mag_field_inertial) # rotate magnetic field into body

        mag_x = mag_field_body[0][0] # get x component of mag field in body

        mag_y = mag_field_body[1][0] # get y component of mag field in body

        mag_z = mag_field_body[2][0] # get z component of mag field in body

        return mag_x, mag_y, mag_z # Return components in body
    
    def updateAccelsTrue(self, state, dot):

        # Function to update the accelerometers using the formulas in Beard Ch 7

        # The accelerometer equations can be found in Ch 7 Pg 122 of Beard

        a_x = dot.u +  (state.q * state.w) - (state.r * state.v) + (VPC.g0 * math.sin(state.pitch)) # Get accelerometer in x reading

        a_y = dot.v + (state.r * state.u) - (state.p * state.w) - (VPC.g0 * math.cos(state.pitch) * math.sin(state.roll)) # Get accelerometer in y reading

        a_z = dot.w + (state.p * state.v) - (state.q * state.u) - (VPC.g0 * math.cos(state.pitch) * math.cos(state.roll)) # Get accelerometer in z reading

        return a_x, a_y, a_z # Return accel readings for each axis x y z
    
    def updateGPSTrue(self, state, dot):

        # Updates GPS state and returns parameters

        gps_N = state.pn # Get GPS Northern coordinate

        gps_E = state.pe # Get GPS Eastern coordinate

        gps_alt = -state.pd # GPS altitude is opposite of down

        gps_SOG = math.hypot(dot.pn, dot.pe) # Get speed over ground for GPS as defined in lecture

        gps_COG = math.atan2(dot.pe, dot.pn) # Get course over ground for GPS as defined in lecture

        return gps_N, gps_E, gps_alt, gps_SOG, gps_COG # Return all GPS true params
    
    def updatePressureSensorsTrue(self, state):

        # Updates Barot and Pitot

        # Get True Pitot Sensor Reading

        Pitot = (VPC.rho * (state.Va ** 2)) / 2 # Beard Pg 130 also found in lecture

        # Get True Baro

        Baro = VSC.Pground + (VPC.rho * VPC.g0 * state.pd) # Given equation from lecture

        return Baro, Pitot # Return Baro and Pitot True
    
    def updateSensorsTrue(self, prevTrueSensors, state, dot):

        ST = Sensors.vehicleSensors() # Create Sensors true container to fill ST "Sensors True"

        # Deal with GPS Update Ticks

        if((self.updateTicks % self.gpsTickUpdate) == 0): # When the GPS has reached the threshold for an update

            ST.gps_n, ST.gps_e, ST.gps_alt, ST.gps_sog, ST.gps_cog = self.updateGPSTrue(state, dot) # Update the GPS True values as normal

        else:

            # Otherwise do the zero hold and heep the previous GPS values

            ST.gps_n = prevTrueSensors.gps_n # north is prev

            ST.gps_e = prevTrueSensors.gps_e # east is prev

            ST.gps_alt = prevTrueSensors.gps_alt # alt is prev

            ST.gps_sog = prevTrueSensors.gps_sog # sog is prev

            ST.gps_cog = prevTrueSensors.gps_cog # cog is prev

        # Update Everything Else as normal

        ST.gyro_x, ST.gyro_y, ST.gyro_z = self.updateGyrosTrue(state) # Update The Gyros

        ST.accel_x, ST.accel_y, ST.accel_z = self.updateAccelsTrue(state, dot) # Update Accelerometers

        ST.mag_x, ST.mag_y, ST.mag_z = self.updateMagsTrue(state) # Update Magnetometers

        ST.baro, ST.pitot = self.updatePressureSensorsTrue(state) # Update Baro and Pitot

        return ST # Return filled Sensors True Container
    
    def updateSensorsNoisy(self, trueSensors = Sensors.vehicleSensors(), noisySensors = Sensors.vehicleSensors(), sensorBiases = Sensors.vehicleSensors(), sensorSigmas = Sensors.vehicleSensors()):

        # Gets all True measurements and adds noise and biases to each as applicable

        SN = Sensors.vehicleSensors() # Create empty sensors noisy container SN "Sensors Noisy"


        # Deal With GPS

        if((self.updateTicks % self.gpsTickUpdate) == 0): # When the GPS has reached the threshold for an update

            # Get colored noise from GM

             C_noise_GPSN, C_noise_GPS_E, C_noise_GPS_alt = self.GPS_GM_XYZ.update()

            # Update GPS with colored noise and white noise GPS is unbiased

             SN.gps_n = trueSensors.gps_n + C_noise_GPSN + random.gauss(0, sensorSigmas.gps_n) # Update GPS N

             SN.gps_e = trueSensors.gps_e + C_noise_GPS_E + random.gauss(0, sensorSigmas.gps_e)  # Update GPS E

             SN.gps_alt = trueSensors.gps_alt + C_noise_GPS_alt + random.gauss(0, sensorSigmas.gps_alt)  # Update GPS alt

             SN.gps_sog = trueSensors.gps_sog + random.gauss(0, sensorSigmas.gps_sog) # No Colored noise for sog

            # Deal with and Trap COG

             if(math.isclose(0, trueSensors.gps_sog)): # If numerator of trap term is near or at 0 we need to trap it
                 
                 SN.gps_cog = trueSensors.gps_cog + random.gauss(0, sensorSigmas.gps_cog) # If trap needed just use sigma COG

             else:
                 
                 Trap_term = (sensorSigmas.gps_cog * VPC.InitialSpeed) / trueSensors.gps_sog # Term to check and trap if necessary from lecture
                 
                 SN.gps_cog = trueSensors.gps_cog + random.gauss(0, Trap_term) # Otherwise use the trap term

            # Wrap within Pi

             if(SN.gps_cog > math.pi):
                 
                 SN.gps_cog = math.pi

             if(SN.gps_cog < -math.pi):
                 
                 SN.gps_cog = -math.pi

        else: # Do the Zero hold again for GPS

            SN.gps_n = noisySensors.gps_n # north is prev

            SN.gps_e = noisySensors.gps_e # east is prev

            SN.gps_alt = noisySensors.gps_alt # alt is prev

            SN.gps_sog = noisySensors.gps_sog # sog is prev

            SN.gps_cog = noisySensors.gps_cog # cog is prev
        
        # Add noise to everything else

        # Update Gyros

        # Colored Noise for Gyro

        C_gx, C_gy, C_gz = self.Gyro_GM_XYZ.update() # Get GM colored noise

        SN.gyro_x = trueSensors.gyro_x + sensorBiases.gyro_x + C_gx + random.gauss(0, sensorSigmas.gyro_x) # Add noise gyro x

        SN.gyro_y = trueSensors.gyro_y + sensorBiases.gyro_y + C_gy + random.gauss(0, sensorSigmas.gyro_y) # Add noise gyro y

        SN.gyro_z = trueSensors.gyro_z + sensorBiases.gyro_z + C_gz + random.gauss(0, sensorSigmas.gyro_z) # Add noise gyro z


        # Update Accelerometers No Colored Noise

        SN.accel_x = trueSensors.accel_x + sensorBiases.accel_x + random.gauss(0, sensorSigmas.accel_x) # Add noise accel x

        SN.accel_y = trueSensors.accel_y + sensorBiases.accel_y + random.gauss(0, sensorSigmas.accel_y) # Add noise accel y

        SN.accel_z = trueSensors.accel_z + sensorBiases.accel_z + random.gauss(0, sensorSigmas.accel_z) # Add noise accel z

        # Update Magnetometers No Colored Noise

        SN.mag_x = trueSensors.mag_x + sensorBiases.mag_x + random.gauss(0, sensorSigmas.mag_x) # Add noise mag x

        SN.mag_y = trueSensors.mag_y + sensorBiases.mag_y + random.gauss(0, sensorSigmas.mag_y) # Add noise mag y

        SN.mag_z = trueSensors.mag_z + sensorBiases.mag_z + random.gauss(0, sensorSigmas.mag_z) # Add noise mag z

        # Update Baro and Pitot Again No colored Noise

        SN.baro = trueSensors.baro  + sensorBiases.baro + random.gauss(0, sensorSigmas.baro) # Add noise to Barot

        SN.pitot = trueSensors.pitot  + sensorBiases.pitot + random.gauss(0, sensorSigmas.pitot) # Add noise to Pitot

        return SN # Return sensors Noisy
    
    def update(self):

        state = self.VAM.VDynamics.state # Get current state

        dot = self.VAM.VDynamics.dot # get current state derivative

        prevSensorTrue = self.sensorsTrue # The previous sensor true state is the current sensors true state

        self.sensorsTrue = self.updateSensorsTrue(prevSensorTrue, state, dot) # Get updated Sensors True

        self.sensorsNoisy = self.updateSensorsNoisy(self.sensorsTrue, self.sensorsNoisy, self.sensorsBiases, self.sensorsSigmas) # Pass sensors true to sensors noisy

        self.updateTicks += 1 # Increment Tick Counter

        return # return nothing
    
    
    













                 



        


            










































