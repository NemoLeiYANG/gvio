from math import cos
from math import sin
from math import atan2
from math import asin
from math import sqrt
from math import pi

import numpy as np


def euler2rot(euler, euler_seq):
    """Convert euler to rotation matrix R
    This function assumes we are performing a body fixed intrinsic rotation.

    Source:

        Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
        Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
        Princeton University Press, 1999. Print.

        Page 86.

    Parameters
    ----------
    euler : np.array
        Euler angle (roll, pitch, yaw)
    euler_seq : float
        Euler rotation sequence

    Returns
    -------

        Rotation matrix (np.array)

    """
    if euler_seq == 321:  # i.e. ZYX rotation sequence (world to body)
        phi, theta, psi = euler

        R11 = cos(psi) * cos(theta)
        R21 = sin(psi) * cos(theta)
        R31 = -sin(theta)

        R12 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi)
        R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi)
        R32 = cos(theta) * sin(phi)

        R13 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)
        R23 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)
        R33 = cos(theta) * cos(phi)

    elif euler_seq == 123:  # i.e. XYZ rotation sequence (body to world)
        phi, theta, psi = euler

        R11 = cos(psi) * cos(theta)
        R21 = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi)
        R31 = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi)

        R12 = sin(psi) * cos(theta)
        R22 = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi)
        R32 = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi)

        R13 = -sin(theta)
        R23 = cos(theta) * sin(phi)
        R33 = cos(theta) * cos(phi)

    else:
        err_msg = "Error! Unsupported euler sequence [%s]" % str(euler_seq)
        raise RuntimeError(err_msg)

    return np.array([[R11, R12, R13],
                     [R21, R22, R23],
                     [R31, R32, R33]])


def quatnorm(q):
    """Norm of JPL quaternion

    Parameters
    ----------
    q :
        np

    Returns
    -------
    type
        Norm of quaternion (float)

    """
    q1, q2, q3, q4 = q.ravel()
    return sqrt(sum(x**2 for x in q))


def quatnormalize(q):
    """Normalize JPL quaternion

    Parameters
    ----------
    q :
        np

    Returns
    -------
    type
        Normalized quaternion (np.array - 4x1)

    """
    q1, q2, q3, q4 = q.ravel()
    mag = quatnorm(q)
    q1 = q1 / mag
    q2 = q2 / mag
    q3 = q3 / mag
    q4 = q4 / mag
    return np.array([[q1], [q2], [q3], [q4]])


def quat2euler(q):
    """JPL Quaternion to Euler angles"""
    x, y, z, w = q.ravel()

    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = atan2(t3, t4)

    return np.array([[X], [Y], [Z]])


def euler2quat(euler):
    """Euler angles to JPL Quaternion"""
    roll, pitch, yaw = euler.ravel()

    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)

    q = np.array([[cy * sr * cp - sy * cr * sp],
                  [cy * cr * sp + sy * sr * cp],
                  [sy * cr * cp - cy * sr * sp],
                  [cy * cr * cp + sy * sr * sp]])

    return quatnormalize(q)


def quat2rot(q):
    """JPL Quaternion to rotation matrix

    Source:

        Page 9.

        Trawny, Nikolas, and Stergios I. Roumeliotis. "Indirect Kalman filter
        for 3D attitude estimation." University of Minnesota, Dept. of Comp.
        Sci. & Eng., Tech. Rep 2 (2005): 2005.

    Parameters
    ----------
    q :
        np

    Returns
    -------
    type
        Product of quaternion multiplication

    """
    q1, q2, q3, q4 = q.ravel()

    R11 = 1.0 - 2.0 * q2**2.0 - 2.0 * q3**2.0
    R12 = 2.0 * (q1 * q2 + q3 * q4)
    R13 = 2.0 * (q1 * q3 - q2 * q4)

    R21 = 2.0 * (q1 * q2 - q3 * q4)
    R22 = 1.0 - 2.0 * q1**2.0 - 2.0 * q3**2.0
    R23 = 2.0 * (q2 * q3 + q1 * q4)

    R31 = 2.0 * (q1 * q3 + q2 * q4)
    R32 = 2.0 * (q2 * q3 - q1 * q4)
    R33 = 1.0 - 2.0 * q1**2.0 - 2.0 * q2**2.0

    return np.array([[R11, R12, R13],
                     [R21, R22, R23],
                     [R31, R32, R33]])


# q_C0G = np.array([0.5, -0.5, 0.5, -0.5])
# C_C0G = quat2rot(q_C0G)

euler = np.array([-pi / 2.0, 0.0, -pi/2.0])
C_C0G = euler2rot(euler, 123)
C_C1G = euler2rot(euler, 123)
p_G_C0 = np.array([0.0, 0.0, 0.0])
p_G_C1 = np.array([0.2, 0.0, 0.0])

C_C0C1 = np.dot(C_C0G, C_C1G.T)
t_C0_C1C0 = np.dot(C_C0G, (p_G_C1 - p_G_C0))

T_C0_C1 = np.block([[C_C0C1, t_C0_C1C0.reshape((3, 1))],
                    [0.0, 0.0, 0.0, 1.0]])

# print(np.round(t_C0_C1C0, 4))

p_C1_f = np.array([0.0, 0.0, 10.0, 1.0])
print(np.round(np.dot(T_C0_C1, p_C1_f), 4))

p_C0_f = np.array([0.0, 0.0, 10.0, 1.0])
T_C1_C0 = np.linalg.inv(T_C0_C1)
print(np.round(np.dot(T_C1_C0, p_C0_f), 4))

# print(np.round(np.dot(C_C0G, (p_G_C1 - p_G_C0)), 4))
# print(np.round(T_C0_C1, 4))
