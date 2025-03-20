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

            self.estState = States.vehicleState()

            self.estState.pd = -VPC.InitialDownPosition

            self.estState.Va = VPC.InitialSpeed

            self.R_hat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

            self.Va_hat = 0.0

            self.b_hat_Va = 0.0

            # Intialize Low Pass Filter for Baro

            self.BaroLPF = LowPassFilter()

            # Intialize Biases for each filter

            self.estimatedGyroBias = [[0], [0], [0]]

            self.estimatedPitotBias = 0

            self.estimatedChiBias = 0

            self.estimatedAscentRate = 0

            self.estimatedAltitudeGPSBias = 0
        
            return # return nothing
        

        def getEstimatedState(self):
             
             return self.estState # Return est state
        
        def setEstimatedState(self, estimatedState = States.vehicleState()):
             
             self.estState = estimatedState
             
             return
        
        def getEstimatorGains(self):
             
             return self.gains
        
        def setEstimatorGains(self, gains = Controls.VehicleEstimatorGains()):
             
             self.gains = gains
             

             return
        
        def setEstimatorBiases(self, estimatedGyroBias = [[0], [0], [0]], estimatedPitotBias = 0, estimatedChiBias = 0, estimatedAscentRate = 0, estimatedAltitudeGPSBias = 0):
             
            self.estimatedGyroBias = estimatedGyroBias

            self.estimatedPitotBias = estimatedPitotBias

            self.estimatedAltitudeGPSBias = estimatedAltitudeGPSBias 

            self.estimatedChiBias = estimatedChiBias

            self.estimatedAscentRate = estimatedAscentRate


            return
        

        
        def reset(self):
             
             self.BaroLPF.reset()

             self.estimatedState = States.vehicleState()

             self.estimatedGyroBias = [[0], [0], [0]]

             self.estimatedPitotBias = 0

             self.estimatedChiBias = 0

             self.estimatedAscentRate = 0

             self.estimatedAltitudeGPSBias = 0

             self.estState.pd = VPC.InitialDownPosition

             self.estState.Va = VPC.InitialSpeed

             self.R_hat = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

             return
             

        


        def estimateAttitude(self, sensorData = Sensors.vehicleSensors(), estimatedState = States.vehicleState()):

            # This follows Algorithim 5 from the Estimation handout and the given block diagram for CF Attitude in both lecture and handout

            dT = self.dT # Get dT

            # Get the necessary gains for this filter

            Kp_acc = self.gains.Kp_acc # Proportional gain for accelerometer

            Ki_acc = self.gains.Ki_acc # Integral gain for accelerometer

            Kp_mag = self.gains.Kp_mag # Proportional gain for magnetometer

            Ki_mag = self.gains.Ki_mag # Integral gain for magnetometer

            # Intialize R_hat and b_hat

            # I made R_hat an internal variable so its not needed here

            b_hat = [[self.estimatedGyroBias[0][0]], [self.estimatedGyroBias[1][0]], [self.estimatedGyroBias[2][0]]] # Get gyro biases for all axes

            # Normalize Accel and Mag in inertial frame

            acc_Inertial = mm.vectorNorm([[0], [0], [VPC.g0]])

            mag_Inertial = mm.vectorNorm(VSC.magfield) # Normalize magnetic field

            # Normalize Accel and Mag in Body frame

            acc_Body = mm.vectorNorm([[sensorData.accel_x], [sensorData.accel_y], [sensorData.accel_z]]) # Normalize acc in body frame

            mag_Body = mm.vectorNorm([[sensorData.mag_x], [sensorData.mag_y], [sensorData.mag_z]]) # Normalize mag in body frame

            # Get actual and biased gyros

            gyros_actual = [[sensorData.gyro_x], [sensorData.gyro_y], [sensorData.gyro_z]] # Get the actual gyro readings from the sensors

            gyros_biased = mm.subtract(gyros_actual, b_hat) # Get the biased gyro readings

            # Get cross product terms for mag and acc for the two feedback loops

            w_err_mag = mm.crossProduct(mag_Body, mm.multiply(self.R_hat, mag_Inertial))

            w_err_acc = mm.crossProduct(acc_Body, mm.multiply(self.R_hat, acc_Inertial))

            # Multiply both errors by Kp to go back into gyro (w hat feedback loop)

            kp_w_err_mag = mm.scalarMultiply(Kp_mag, w_err_mag)

            kp_w_err_acc = mm.scalarMultiply(Kp_acc, w_err_acc)

            # Multiply both errors by Ki to go back into sensors (b hat feedback loop)

            ki_w_err_mag = mm.scalarMultiply(-Ki_mag, w_err_mag)

            ki_w_err_acc = mm.scalarMultiply(-Ki_acc, w_err_acc)

            # Check condition acc_body

            magnitude_acc_b = math.hypot(sensorData.accel_x, sensorData.accel_y, sensorData.accel_z)

            if(0.9 * VPC.g0 <= magnitude_acc_b <= 1.1 * VPC.g0):
                 
                 feedback_gyro = mm.add(gyros_biased, mm.add(kp_w_err_mag, kp_w_err_acc))

                 b_dot = mm.add(ki_w_err_mag, ki_w_err_acc)

            else:
                 
                 # Don't use acc not valid

                 feedback_gyro = mm.add(gyros_biased, ki_w_err_mag)

                 b_dot = ki_w_err_mag

            b_hat = mm.add(b_hat, mm.scalarMultiply(dT, b_dot))

            # create dummy dot and dummy state

            dummy_dot = States.vehicleState()

            dummy_state = States.vehicleState()

            dummy_state.p = feedback_gyro[0][0]

            dummy_state.q = feedback_gyro[1][0]

            dummy_state.r = feedback_gyro[2][0]

            matrix_exp = VDM.VehicleDynamicsModel().Rexp(dT, dummy_state, dummy_dot)
            
            R_plus = mm.multiply(matrix_exp, estimatedState.R)

            self.R_hat = R_plus

            return b_hat, gyros_biased, R_plus
        

        def estimateAltitude(self, sensorData = Sensors.vehicleSensors(), estimatedState = States.vehicleState()):
             
             # Intialize and run h_baro through LPF
             
             h_baro = self.BaroLPF.update(sensorData.baro) # Get h_baro

             # Get a up inertial

             R_hat_transpose = mm.transpose(self.R_hat) # Transpose R_hat

             acc_body = [[sensorData.accel_x], [sensorData.accel_y], [sensorData.accel_z]] # Get body measured accelerations from accelrometer

             R_hat_T_a_body = mm.multiply(R_hat_transpose, acc_body) # Multiply R_hat transpose by body accelerations

             # Extract [3, 1] element

             R_hat_element = R_hat_T_a_body[2][0] # Get [3, 1] element of R_hat transpose

             a_up_inert = R_hat_element + VPC.g0 # get upwards inertial acceleration

             # Get Timestep 

             dT = self.dT


             return 1, 1, 1
        

        def estimateAirspeed(self, sensorData = Sensors.vehicleSensors(), estimatedState = States.vehicleState()):
             
             # This follows algorithim 2 of the handout and its asscoiated CF Block Diagram
             
             # Get timestep

             dT = self.dT # initialize timestep
             
             # Get necessary gains

             Kp_Va = self.gains.Kp_Va # Proportional gain for airspeed

             Ki_Va = self.gains.Ki_Va # Integral Gain for airspeed

             # Get Va Pitot

             Va_pitot = math.sqrt(2 * sensorData.pitot / VPC.rho) # Va pitot equation from handout

             # Initialize estimate and Bias

             Va_hat = estimatedState.Va # Get estimated Va hat

             b_hat_Va = 0.0 # Estimated bias

             # Get ax term

             grav_vector = [[0], [0], [VPC.g0]] # gravity vector for ax

             acc_body = [[sensorData.accel_x], [sensorData.accel_y], [sensorData.accel_z]] # body acceleration in each axis from accelerometer

             ax = mm.add(acc_body, mm.multiply(estimatedState.R, grav_vector )) # Get acceleration vector

             ax_extract = ax[0][0]  # Extract [1, 1] element from acceleration vector

             # If measurement Update the airspeed

             b_hat_dot = -Ki_Va * (Va_pitot - Va_hat) # b_hat_dot eqaution

             b_hat_Va = b_hat_Va + (b_hat_dot * dT) # b_hat_Va equation integrate bias

             term_to_add = -b_hat_Va + (Kp_Va * (Va_pitot - Va_hat)) # Piece added to get Va dot and incoorporate ax
             
             Va_dot = ax_extract + term_to_add # Incorporate ax

             Va_hat = Va_hat + (Va_dot * dT) # Get Va hat

             return b_hat_Va, Va_hat # Return b_hat_Va and Va_hat



             














                





            

    



