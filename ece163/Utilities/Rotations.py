import math
from . import MatrixMath



def ned2enu(points):

    MTRX_TO_ENU = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]

    result = MatrixMath.multiply(MTRX_TO_ENU, MatrixMath.transpose(points))

    result = MatrixMath.transpose(result)

    return result

def euler2DCM (yaw, pitch, roll):

    
    DCM = [[(math.cos(pitch) * math.cos(yaw)), (math.cos(pitch) * math.sin(yaw)), (-1*math.sin(pitch))],
           [((math.sin(roll) * math.sin(pitch) * math.cos(yaw)) - (math.cos(roll) * math.sin(yaw))), ((math.sin(roll) * math.sin(pitch) * math.sin(yaw)) + (math.cos(roll) * math.cos(yaw))), (math.sin(roll) * math.cos(pitch))],
           [((math.cos(roll) * math.sin(pitch) * math.cos(yaw)) + (math.sin(roll) * math.sin(yaw))), ((math.cos(roll) * math.sin(pitch) * math.sin(yaw)) - (math.sin(roll) * math.cos(yaw))), (math.cos(roll) * math.cos(pitch))]]

    #DCM = math.cos(yaw) * math.cos(pitch) * math.cos(roll)

    #MatrixMath.matrixPrint(DCM)


    return DCM


def dcm2Euler (DCM)
    
    yaw = math.atan2(DCM[1][2], DCM[1][1])

    pitch = math.atan2(DCM[2][3], DCM[3][3])

    roll = math.asin(DCM[1][3])

    if roll >= 1:

        roll = 1

    elif roll <= -1:

        roll = -1

        return yaw, pitch, roll
