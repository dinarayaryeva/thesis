import sympy as sp


def skew(v):
    return sp.Matrix([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def quaternion_product(q):
    q0, q1, q2, q3 = q
    return 0.5*sp.Matrix([[-q1, -q2, -q3],
                          [q0, -q3, q2],
                          [q3, q0, -q1],
                          [-q2, q1, q0]])


def quat2rot(q):
    # Extract the values
    q0, q1, q2, q3 = q

    # First row of the rotation matrix
    r00 = q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = q0 ** 2 - q1 ** 2 + q2 ** 2 - q3 ** 2
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2

    # 3x3 rotation matrix
    rot_matrix = sp.Matrix([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22],
                            ])

    return rot_matrix
