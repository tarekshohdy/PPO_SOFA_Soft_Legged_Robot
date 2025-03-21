import numpy as np

def matrix_B(point1: list, point2: list, point3: list) -> np.ndarray:
    """
    Calculate the matrix B for a triangle defined by three points
    """
    B = np.array(
        [[point1[0], point1[1], point1[2]],
        [point2[0], point2[1], point2[2]],
        [point3[0], point3[1], point3[2]]
        ])
    
    return B

def calculate_roll_pitch_yaw (B: np.ndarray, point1: list, point2: list, point3: list) -> list:

    X1 = np.array([point1[0], point2[0], point3[0]])
    Y1 = np.array([point1[1], point2[1], point3[1]])
    Z1 = np.array([point1[2], point2[2], point3[2]])

    B_inv = np.linalg.inv(B)

    a = np.dot(B_inv, X1)[0]
    b = np.dot(B_inv, Y1)[0]
    m = np.dot(B_inv, Z1)

    c = m[0]
    d = m[1]
    e = m[2]

    pitch = np.arcsin(-c)
    yaw = np.arctan2(b, a)
    roll = np.arctan2(d, e)

    return [roll, pitch, yaw]
