from geometry_msgs.msg import Quaternion

import numpy as np
from numpy.typing import NDArray

def rotmat2q(T: NDArray) -> Quaternion:
    # Function that transforms a 3x3 rotation matrix to a ros quaternion representation

    if T.shape != (3, 3):
        raise ValueError("the input matrix must be 3x3 !")

    m00, m01, m02 = T[0, 0], T[0, 1], T[0, 2]
    m10, m11, m12 = T[1, 0], T[1, 1], T[1, 2]
    m20, m21, m22 = T[2, 0], T[2, 1], T[2, 2]

    qw = np.sqrt(1 + m00 + m11 + m22) / 2
    qx = (m21 - m12) / (4 * qw)
    qy = (m02 - m20) / (4 * qw)
    qz = (m10 - m01) / (4 * qw)

    q = Quaternion()
    q.x = qx
    q.y = qy
    q.z = qz
    q.w = qw
    return q