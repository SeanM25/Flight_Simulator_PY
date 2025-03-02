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

def computeGains(tuningParamters = Controls.controlTuning(), linearizedModel = Linearized.transferFunctions()):


    # Function computes Lateral & Longitudal Gains as outlined in Beard Ch 6

    # Fills and returns a control gains object with the appropriate parameters


    controlGains = Controls.controlGains() # Create an empty Control Gains instance to fill and return

    # Roll Gains

    Wn_roll = tuningParamters.Wn_roll # Get roll frequency

    a_phi1 = linearizedModel.a_phi1 # Get a_phi 1

    a_phi2 = linearizedModel.a_phi2 # Get a_phi 2
    
    Zeta_roll = tuningParamters.Zeta_roll # Get zeta roll

    controlGains.kp_roll = ((Wn_roll ** 2) / a_phi2 ) # This is Beard's equation (6.8) sub into (6.7)

    controlGains.kd_roll = ((2 * Zeta_roll * Wn_roll) - a_phi1) / a_phi2 # Beard (6.9)
    
    controlGains.ki_roll = 0.001 # No ki term in roll autopilot

    # Sideslip Gains

    Wn_sideslip = tuningParamters.Wn_sideslip # Get sideslip frequency

    a_beta1 = linearizedModel.a_beta1 # Get a_beta 1

    a_beta2 = linearizedModel.a_beta2 # Get a_beta 2

    Zeta_sideslip = tuningParamters.Zeta_sideslip # Get zeta sideslip

    controlGains.kp_sideslip = ((2 * Zeta_sideslip * Wn_sideslip) - a_beta1) / a_beta2 # Beard (6.15)

    controlGains.ki_sideslip = (Wn_sideslip ** 2) / a_beta2 # Beard (6.14)

    # Course Gains 

    Vg = linearizedModel.Va_trim # From lecture

    Wn_course = tuningParamters.Wn_course # Get course frequency

    Zeta_course = tuningParamters.Zeta_course # Get zeta course

    controlGains.kp_course = (2 * Zeta_course * Wn_course * Vg) / VPC.g0 # Beard (6.12)

    controlGains.ki_course = ((Wn_course ** 2) * Vg) / VPC.g0 # Beard (6.13)

    # Pitch Gains

    Wn_pitch = tuningParamters.Wn_pitch # Get pitch frequency

    Zeta_pitch = tuningParamters.Zeta_pitch # Get Zeta pitch

    a_theta1 = linearizedModel.a_theta1 # Get a_theta 1

    a_theta2 = linearizedModel.a_theta2 # Get a_theta 2

    a_theta3 = linearizedModel.a_theta3 # Get a_theta 3

    controlGains.kp_pitch = ((Wn_pitch ** 2) - a_theta2) / a_theta3 # Beard (6.19)

    controlGains.kd_pitch = ((2 * Zeta_pitch * Wn_pitch) - a_theta1) / a_theta3 # Beard (6.22)

    # Altitude Gains

    k_theta_dc = (controlGains.kp_pitch * a_theta3) / (a_theta2 + (controlGains.kp_pitch * a_theta3)) # Beard (6.23)

    Zeta_altitude = tuningParamters.Zeta_altitude # Geta zeta for altitude

    Va = linearizedModel.Va_trim # Get Va

    Wn_altitude = tuningParamters.Wn_altitude # Get frequency for altitude gain

    controlGains.kp_altitude = (2 * Zeta_altitude * Wn_altitude) / (k_theta_dc * Va) # Beard (6.25)

    controlGains.ki_altitude = (Wn_altitude ** 2) / (k_theta_dc * Va) # Beard (6.24)

    # Speed From Throttle Gains

    a_V1 = linearizedModel.a_V1 # Get a_V1

    a_V2 = linearizedModel.a_V2 # Get a_V2

    a_V3 = linearizedModel.a_V3 # Get a_V3

    Zeta_SpeedfromThrottle = tuningParamters.Zeta_SpeedfromThrottle # Get Zeta Airspeed Hold Using Throttle

    Wn_SpeedfromThrottle = tuningParamters.Wn_SpeedfromThrottle # Get Freq Airspeed Hold Using Throttle

    controlGains.kp_SpeedfromThrottle = (( 2 * Zeta_SpeedfromThrottle * Wn_SpeedfromThrottle) - a_V1) / a_V2 # Beard (6.30)

    controlGains.ki_SpeedfromThrottle = (Wn_SpeedfromThrottle ** 2) / a_V2 # Beard (6.29)

    # Speed From Elevator Gains

    Wn_SpeedfromElev = tuningParamters.Wn_SpeedfromElevator # Get freq for SFE

    Zeta_SpeedfromElev = tuningParamters.Zeta_SpeedfromElevator # Get Zeta for SFE

    controlGains.ki_SpeedfromElevator = -1 * ((Wn_SpeedfromElev ** 2) / (k_theta_dc * VPC.g0)) # Beard (6.27)

    controlGains.kp_SpeedfromElevator = (a_V1 - (2 * Zeta_SpeedfromElev * Wn_SpeedfromElev)) / (k_theta_dc * VPC.g0) # Beard (6.28)

    return controlGains # Return all calculated gains



def computeTuningParameters(controlGains = Controls.controlGains(), linearizedModel = Linearized.transferFunctions()):

    # Opposite of compute gains we compute and return the tuning parameters here

    # Tuning Parameters from Ch 6 of Beard

    tuningParameters = Controls.controlTuning()

    Vg = linearizedModel.Va_trim

    # Race Condition Preventing Function From passing calculate equation params before hand?

    kp_roll = controlGains.kp_roll

    a_phi2 = linearizedModel.a_phi2

    kd_roll = controlGains.kd_roll

    a_phi1 = linearizedModel.a_phi1

    ki_course = controlGains.ki_course

    kp_course = controlGains.kp_course

    ki_sideslip = controlGains.ki_sideslip

    a_beta1 = linearizedModel.a_beta1

    a_beta2 = linearizedModel.a_beta2

    kp_sideslip = controlGains.kp_sideslip

    a_theta1 = linearizedModel.a_theta1

    a_theta2 = linearizedModel.a_theta2

    a_theta3 = linearizedModel.a_theta3

    kp_pitch = controlGains.kp_pitch

    kd_pitch = controlGains.kd_pitch

    a_V1 = linearizedModel.a_V1

    a_V2 = linearizedModel.a_V2

    a_V3 = linearizedModel.a_V3

    ki_altitude = controlGains.ki_altitude

    kp_altitude = controlGains.kp_altitude

    ki_SpeedfromThrottle = controlGains.ki_SpeedfromThrottle

    kp_SpeedfromThrottle = controlGains.kp_SpeedfromThrottle

    ki_SpeedfromElevator = controlGains.ki_SpeedfromElevator

    kp_SpeedfromElevator = controlGains.ki_SpeedfromElevator


    # Roll Tuning Params

    tuningParameters.Wn_roll = math.sqrt(kp_roll * a_phi2)

    Wn_roll = tuningParameters.Wn_roll

    tuningParameters.Zeta_roll = ((kd_roll * a_phi2) + a_phi1) / (2 * Wn_roll)

    # Course Tuning Params
        
    tuningParameters.Wn_course = math.sqrt((VPC.g0 * ki_course)/ Vg )

    Wn_course = tuningParameters.Wn_course

    tuningParameters.Zeta_course = (kp_course * VPC.g0) / (2 * Wn_course * Vg)

    # Sideslip Tuning Params
        
    tuningParameters.Wn_sideslip = math.sqrt(a_beta2 * ki_sideslip)

    Wn_sideslip = tuningParameters.Wn_sideslip

    tuningParameters.Zeta_sideslip = (a_beta1 + (a_beta2 * kp_sideslip)) / (2 * Wn_sideslip)

    # Pitch Tuning Params

    tuningParameters.Wn_pitch = math.sqrt(a_theta2 + (kp_pitch * a_theta3))

    k_theta_dc = (kp_pitch * a_theta3) / (Wn_pitch ** 2)

    Wn_pitch = tuningParameters.Wn_pitch

    tuningParameters.Zeta_pitch = (a_theta1 + (kd_pitch * a_theta3)) / (2 * Wn_pitch)

    # Altitude Tuning Params
        
    tuningParameters.Wn_altitude = math.sqrt(k_theta_dc * Vg * ki_altitude)

    Wn_altitude = tuningParameters.Wn_altitude

    tuningParameters.Zeta_altitude = (k_theta_dc * Vg * kp_altitude) / (2 * Wn_altitude)

    # Speed From Throttle Tuning Params
        
    tuningParameters.Wn_SpeedfromThrottle = math.sqrt(a_V2 * ki_SpeedfromThrottle)

    Wn_SpeedfromThrottle = tuningParameters.Wn_SpeedfromThrottle

    tuningParameters.Zeta_SpeedfromThrottle = (a_V1 + (a_V2 * kp_SpeedfromThrottle)) / (2 * Wn_SpeedfromThrottle)

        # Speed From Elevator Tuning Params
        
    tuningParameters.Wn_SpeedfromElevator = math.sqrt(-1 * k_theta_dc * VPC.g0 * ki_SpeedfromElevator)

    Wn_SpeedfromElevator = tuningParameters.Wn_SpeedfromElevator

    tuningParameters.Zeta_SpeedfromElevator = (a_V1 - (k_theta_dc * VPC.g0 * kp_SpeedfromElevator)) / (2 * Wn_SpeedfromElevator)

    return tuningParameters


   # try:

        
        # Possible exceptions: Bad Values & nonsense parameters passed in

    #except ValueError:

        #tuningParameters = Controls.controlTuning() # Create empty tuning Params

        #return tuningParameters # Return empty
    
    #except:

        #tuningParameters = Controls.controlTuning() # Create empty tuning Params

        #return tuningParameters # Return empty







        
    



    



