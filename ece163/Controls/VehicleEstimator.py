import math
from ..Containers import Controls
from ..Containers import Sensors
from ..Containers import States
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleDynamicsModel as VDM
from ..Sensors import SensorsModel
from ..Utilities import MatrixMath as mm


class LowPassFilter:

    def __init__(self, dT = 0.01, cutoff = 1):

        self.dT = dT # Set timestep of the LPF

        self.cutoffDef = 1 # Default cutoff of 1 Hz

        self.cutoff = cutoff # actual cutoff freq Hz

        self.yk = 0.0 # Initial output of LPF is 0

        self.yk_prev = 0.0 # Initial prev state of LPF is 0

        return # return nothing
    

    def reset(self):

        # reset internal storgage variables that is yk and yk_prev

        self.yk = 0.0 # Reset output to zero

        self.yk_prev = 0.0 # Reset prev yk to zero

        return # return nothing
    
    def update(self, input):

        # Update yk low pass filter based on input

        a = 2 * math.pi * self.cutoff # Given a term

        exp_term = math.exp(-a * self.dT) # Given exp term

        # Calculate Low Pass Filter Output

        self.yk = (exp_term * self.yk_prev) + ((1-exp_term) * input) # LPF equation from both handout and homework

        self.yk_prev = self.yk# Update previous state with new output

        return self.yk # return current output
    

class VehicleEstimator:

        def __init__(self, dT = VPC.dT, gains = Controls.VehicleEstimatorGains(), sensorsModel = SensorsModel.SensorsModel()):

            self.sensorsModel = sensorsModel # Create sensors model attribute

            self.dT = dT # Time stamp attribute

            # Initialize Gains for each of the 4 Filters (Attitude, Airspeed, Airspeed, Altitude, Course)

            self.gains = gains # Get gains for each of the filters

            # Initialize Est States for each of the 4 Filters (Attitude, Airspeed, Altitude, Course)

            self.estState = States.vehicleState(u = 9, v = 12, w = 20, pd = -VPC.InitialDownPosition)

            # Intialize Low Pass Filter for Baro

            self.BaroLPF = LowPassFilter()

            # Intialize Biases for each filter

            self.biases = Sensors.vehicleSensors() # Get all biases for each filter
        
            return # return nothing
        

        def getEstimatedState(self):
             
             return self.estState # Return est state
        
        def setEstimatedState(self, estimatedState = States.vehicleState()):
             
             self.estState = estimatedState
             
             return
        
        def getEstimatorGains(self):
             
             return self.filterGains
        
        def setEstimatorGains(self, gains = Controls.VehicleEstimatorGains()):
             
             self.filterGains = gains
             

             return
        
        def setEstimatorBiases(self, estimatedGyroBias = [[0], [0], [0]], estimatedPitotBias = 0, estimatedChiBias = 0, estimatedAscentRate = 0, estimatedAltitudeGPSBias = 0):
             
             self.biases.gyro_x = estimatedGyroBias[0][0]

             self.biases.gyro_y = estimatedGyroBias[1][0]

             self.biases.gyro_z = estimatedGyroBias[2][0]


             self.biases.pitot = estimatedPitotBias

             self.biases.gps_alt = estimatedAltitudeGPSBias 

             self.biases.gps_cog = estimatedChiBias

             self.biases.gps_sog = estimatedAltitudeGPSBias


             return
        



             

        
        
        
        def estimateAttitude(self, sensorData = Sensors.vehicleSensors(), estimatedState = States.vehicleState()):

            # This follows Algorithim 5 from the Estimation handout

            dT = self.dT

            # Intialize IC's and Inertials

            R_hat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] # Intial DCM for Atitude

            b_hat = [[0.0], [0.0], [0.0]] # Intial Bias

            a_hat_i = mm.vectorNorm([[0.0], [0.0], [-VPC.g0]]) # Normalize known inertial acceleration

            h_hat_i = mm.vectorNorm(VSC.magfield) # Normalize known inertial magnetic field

            w_meas = [[sensorData.gyro_x], [sensorData.gyro_y], [sensorData.gyro_z]]

            # Get Body Frame readings for magnetometer and accelerometer

            h_hat_bod = mm.vectorNorm([[sensorData.mag_x], [sensorData.mag_y], [sensorData.mag_z]]) # Get body mag field readings and normalize

            a_hat_bod = mm.vectorNorm([[sensorData.accel_x], [sensorData.accel_y], [sensorData.accel_z]])

            w_error_mag = mm.crossProduct(h_hat_bod, mm.multiply(R_hat, h_hat_i)) # Get angular rates error for magnetometer

            b_hat_dot = mm.scalarMultiply(-self.gains.Ki_mag, w_error_mag) # Get bias estimate rate

            if(0.9*VPC.g0 <= math.hypot(sensorData.accel_x, sensorData.accel_y, sensorData.accel_z) <= 1.1*VPC.g0 ):

                w_error_acc = mm.crossProduct(a_hat_bod, mm.multiply(R_hat, a_hat_i))

                b_hat_dot = b_hat_dot + mm.scalarMultiply(-self.gains.Ki_acc, w_error_acc)
            else:

                b_hat = b_hat + (b_hat_dot * dT)

            # Get w hat R+ and R hat

            # Create dummy dot state

            dummy_dot = States.vehicleState()

            # dummy state

            w_hat = w_meas - b_hat


            return b_hat, w_hat, 1
                





            

    



