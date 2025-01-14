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

