import math
from . import MatrixMath



def ned2enu(points):

    MTRX_TO_ENU = [[0, 1, 0], [1, 0, 0], [0, 0, -1]] # Given NED to ENU conversion matrix

    result = MatrixMath.multiply(MTRX_TO_ENU, MatrixMath.transpose(points)) # Convert NED to ENU coordinates do the transpose to get cols of x, y, z

    result = MatrixMath.transpose(result) # Transpose again to get rows of x, y, z

    return result # return converted coordinate matrix

def euler2DCM (yaw, pitch, roll):

    
    DCM = [[(math.cos(pitch) * math.cos(yaw)), (math.cos(pitch) * math.sin(yaw)), (-1*math.sin(pitch))],
           [((math.sin(roll) * math.sin(pitch) * math.cos(yaw)) - (math.cos(roll) * math.sin(yaw))), ((math.sin(roll) * math.sin(pitch) * math.sin(yaw)) + (math.cos(roll) * math.cos(yaw))), (math.sin(roll) * math.cos(pitch))],
           [((math.cos(roll) * math.sin(pitch) * math.cos(yaw)) + (math.sin(roll) * math.sin(yaw))), ((math.cos(roll) * math.sin(pitch) * math.sin(yaw)) - (math.sin(roll) * math.cos(yaw))), (math.cos(roll) * math.cos(pitch))]]
    
    # Given DCM matrix from class. We fill in each point with the appropriate trig operration given yaw, pitch, and roll euler angles

    return DCM # Return DCM


def dcm2Euler (DCM):

    # Bound checking for arcsin(pitch)

    if DCM[0][2] > 1: # If greater than 1

        DCM[0][2] = 1 # Make it 1

    elif DCM[0][2] < -1: # If less than 1

        DCM[0][2] = -1 # Make -1
    
    yaw = math.atan2(DCM[0][1], DCM[0][0]) # Yaw formula from class

    pitch = -1 * math.asin(DCM[0][2]) # Typo in lecture should be a negative arcsin?

    roll = math.atan2(DCM[1][2], DCM[2][2]) # Roll formula
    
    return yaw, pitch, roll # Return Euler Angles
