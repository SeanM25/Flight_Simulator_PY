import math
from . import MatrixMath



def ned2enu(points):

    MTRX_NED2ENU = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]

    NED2ENU_FINAL = MTRX_NED2ENU * MatrixMath.transpose(points)

    NED2ENU_FINAL = MatrixMath.transpose(NED2ENU_FINAL)

    return NED2ENU_FINAL

def euler2DCM (yaw, pitch, roll):


    return 0

