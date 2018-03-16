from math import cos
from math import sin
from math import pi

import numpy as np
from numpy import dot

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # NOQA


def euler321ToRot(euler):
    """Convert euler to rotation matrix R

    Source:

        Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
        Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
        Princeton University Press, 1999. Print.

        Page 86.

    Parameters
    ----------
    euler : np.array
        Euler angle (roll, pitch, yaw)

    Returns
    -------

        Rotation matrix (np.array)

    """
    # i.e. ZYX rotation sequence (world to body)
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

    return np.array([[R11, R12, R13],
                     [R21, R22, R23],
                     [R31, R32, R33]])


def euler123ToRot(euler):
    """Convert euler to rotation matrix R

    Source:

        Kuipers, Jack B. Quaternions and Rotation Sequences: A Primer with
        Applications to Orbits, Aerospace, and Virtual Reality. Princeton, N.J:
        Princeton University Press, 1999. Print.

        Page 86.

    Parameters
    ----------
    euler : np.array
        Euler angle (roll, pitch, yaw)

    Returns
    -------

        Rotation matrix (np.array)

    """
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

    return np.array([[R11, R12, R13],
                     [R21, R22, R23],
                     [R31, R32, R33]])


def dh_transform(theta, d, a, alpha):
    """ Denavit–Hartenberg transform matrix

    Parameters
    ----------
    theta : float
        Angle (radians)
    alpha : float
        Angle (radians)
    a : float
        Offset (m)
    d : float
        Offset (m)

    Returns
    -------
    DH Transform matrix

    """
    c = cos
    s = sin

    return np.array([
        [c(theta), -s(theta) * c(alpha), s(theta) * s(alpha), a * c(theta)],
        [s(theta), c(theta) * c(alpha), -c(theta) * s(alpha), a * s(theta)],
        [0.0, s(alpha), c(alpha), d],
        [0.0, 0.0, 0.0, 1.0],
    ])


class GimbalModel:
    def __init__(self, **kwargs):
        self.attitude = np.array([0.0, 0.0])

        # 6-dof transform from static camera to base mechanism frame
        self.tau_s = kwargs.get(
            "tau_s",
            np.array([-0.045, -0.085, 0.08, 0.0, 0.0, 0.0])
        )

        # 6-dof transform from end-effector frame to dynamic camera
        self.tau_d = kwargs.get(
            "tau_d",
            np.array([0.0, 0.0, 0.0, pi / 2.0, 0.0, -pi / 2.0])
        )

        # DH-params (theta, alpha, a, d)
        self.link1 = kwargs.get("link1", np.array([0.0, 0.1, 0.0, pi / 2.0]))
        self.link2 = kwargs.get("link2", np.array([0.0, 0.0, 0.0, 0.0]))

    def set_attitude(self, attitude):
        self.attitude = attitude

    def T_bs(self, tau_s):
        """ Form transform matrix from static camera to base mechanism

        Parameters
        ----------
        tau_s : np.array
            Parameterization of the transform matrix where the first 3 elements
            in the vector is the translation from static camera to base frame.
            Second 3 elements is rpy, which is the roll pitch yaw from static
            camera to base frame.

        Returns
        -------
        T_bs : np.array
            Transform matrix from static camera to base_frame

        """
        # Setup
        t = tau_s[0:3]
        rpy = tau_s[3:6]
        R = euler321ToRot(rpy)

        # Create base frame
        T_bs = np.array([[R[0, 0], R[0, 1], R[0, 2], t[0]],
                         [R[1, 0], R[1, 1], R[1, 2], t[1]],
                         [R[2, 0], R[2, 1], R[2, 2], t[2]],
                         [0.0, 0.0, 0.0, 1.0]])

        return T_bs

    def T_de(self, tau_d):
        """ Form transform matrix from end-effector to dynamic camera

        Parameters
        ----------
        tau_s : np.array
            Parameterization of the transform matrix where the first 3 elements
            in the vector is the translation from end effector to dynamic
            camera frame. Second 3 elements is rpy, which is the roll pitch yaw
            from from end effector to dynamic camera frame.

        Returns
        -------
        T_de : np.array
            Transform matrix from end effector to dynamic camera

        """
        # Setup
        t = tau_d[0:3]
        R = euler321ToRot(tau_d[3:6])

        # Create transform
        T_de = np.array([[R[0, 0], R[0, 1], R[0, 2], t[0]],
                         [R[1, 0], R[1, 1], R[1, 2], t[1]],
                         [R[2, 0], R[2, 1], R[2, 2], t[2]],
                         [0.0, 0.0, 0.0, 1.0]])

        return T_de

    def T_eb(self, link1, link2):
        """ Form transform matrix from base_frame to end-effector

        Parameters
        ----------
        link1 : np.array
            DH parameters (theta, alpha, a, d) for the first link
        link2 : np.array
            DH parameters (theta, alpha, a, d) for the second link

        Returns
        -------
        T_eb : np.array
            Transform matrix from base frame to end-effector

        """
        theta1, d1, a1, alpha1 = link1
        theta2, d2, a2, alpha2 = link2
        T_1b = np.linalg.inv(dh_transform(theta1, d1, a1, alpha1))
        T_e1 = np.linalg.inv(dh_transform(theta2, d2, a2, alpha2))
        return np.dot(T_e1, T_1b)

    def T_ds(self):
        # Transform from static camera to base frame
        T_bs = self.T_bs(self.tau_s)

        # Transform from base frame to end-effector
        T_eb = self.T_eb(self.link1, self.link2)

        # Transform from end-effector to dynamic camera
        T_de = self.T_de(self.tau_d)

        # Create transforms
        T_es = dot(T_eb, T_bs)  # Transform static camera to end effector
        T_ds = dot(T_de, T_es)  # Transform static camera to dynamic camera

        return T_ds

    def calc_transforms(self):
        # Transform from static camera to base frame
        T_bs = self.T_bs(self.tau_s)

        # Transform from base frame to end-effector
        T_eb = self.T_eb(self.link1, self.link2)

        # Transform from end-effector to dynamic camera
        T_de = self.T_de(self.tau_d)

        # Create links
        links = [T_bs, T_eb, T_de]

        return links


def axis_equal_3dplot(ax):
    extents = np.array([getattr(ax, 'get_{}lim'.format(dim))()
                        for dim in 'xyz'])
    sz = extents[:, 1] - extents[:, 0]
    centers = np.mean(extents, axis=1)
    maxsize = max(abs(sz))
    r = maxsize / 2
    for ctr, dim in zip(centers, 'xyz'):
        getattr(ax, 'set_{}lim'.format(dim))(ctr - r, ctr + r)


class PlotGimbal:
    """ Gimbal plot

    Attributes:
    -----------
    origin : np.array
        Gimbal origin

    attitude : np.array
        Roll, pitch, yaw

    """
    def __init__(self, **kwargs):
        self.origin = np.array([0.0, 0.0, 0.0])
        self.gimbal = kwargs.get("gimbal", GimbalModel())

        self.link0 = None
        self.link1 = None
        self.link2 = None
        self.link3 = None

        self.show_static_frame = kwargs.get("show_static_frame", True)
        self.show_base_frame = kwargs.get("show_base_frame", True)
        self.show_end_frame = kwargs.get("show_end_frame", True)
        self.show_dynamic_frame = kwargs.get("show_dynamic_frame", True)

    def set_attitude(self, attitude):
        self.gimbal.set_attitude(attitude)

    def plot_coord_frame(self, ax, T, frame, length=0.1):
        """ Plot coordinate frame

        Parameters
        ----------
        ax : matplotlib.axes.Axes
            Plot axes

        T : np.array (4x4)
            Transform matrix

        length : float (default: 0.1)
            Length of each coordinate frame axis

        """
        axis_x = dot(T, np.array([length, 0.0, 0.0, 1.0]))
        axis_y = dot(T, np.array([0.0, length, 0.0, 1.0]))
        axis_z = dot(T, np.array([0.0, 0.0, length, 1.0]))

        ax.text(T[0, 3], T[1, 3], T[2, 3] + 0.005, frame, color='Black')

        ax.plot([T[0, 3], axis_x[0]],
                [T[1, 3], axis_x[1]],
                [T[2, 3], axis_x[2]], color="red")
        ax.plot([T[0, 3], axis_y[0]],
                [T[1, 3], axis_y[1]],
                [T[2, 3], axis_y[2]], color="green")
        ax.plot([T[0, 3], axis_z[0]],
                [T[1, 3], axis_z[1]],
                [T[2, 3], axis_z[2]], color="blue")

    def plot(self, ax=None):
        """ Plot gimbal

        Parameters
        ----------
        ax : matplotlib.axes.Axes
            Plot axes

        """
        if ax is None:
            fig = plt.figure()
            ax = fig.gca(projection='3d')
            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_zlabel("z")

        # Create transform from global origin to static camera
        t_g_sg = np.array([0.0, 0.0, 0.0])
        R_sg = euler123ToRot([-pi / 2.0, 0.0, -pi / 2.0])
        T_sg = np.array([[R_sg[0, 0], R_sg[0, 1], R_sg[0, 2], t_g_sg[0]],
                         [R_sg[1, 0], R_sg[1, 1], R_sg[1, 2], t_g_sg[1]],
                         [R_sg[2, 0], R_sg[2, 1], R_sg[2, 2], t_g_sg[2]],
                         [0.0, 0.0, 0.0, 1.0]])

        # Calculate gimbal transforms
        T_bs, T_eb, T_de = self.gimbal.calc_transforms()

        # Plot static camera frame
        length = 0.01
        if self.show_static_frame:
            T_gs = np.linalg.inv(T_sg)
            self.plot_coord_frame(ax, T_gs, "S", length=length)

        # Plot base mechanism frame
        if self.show_base_frame:
            T_bg = dot(T_bs, T_sg)
            T_gb = np.linalg.inv(T_bg)
            self.plot_coord_frame(ax, T_gb, "B", length=length)

        # Plot end effector frame
        if self.show_end_frame:
            T_eg = dot(T_eb, dot(T_bs, T_sg))
            T_ge = np.linalg.inv(T_eg)
            self.plot_coord_frame(ax, T_ge, "E", length=length)

        # Plot dynamic camera frame
        if self.show_dynamic_frame:
            T_dg = dot(T_de, dot(T_eb, dot(T_bs, T_sg)))
            T_gd = np.linalg.inv(T_dg)
            self.plot_coord_frame(ax, T_gd, "D", length=length)

        # Plot links
        self.link0 = ax.plot([0, T_gs[0, 3]],
                             [0, T_gs[1, 3]],
                             [0, T_gs[2, 3]],
                             '--', color="black")
        self.link1 = ax.plot([T_gs[0, 3], T_gb[0, 3]],
                             [T_gs[1, 3], T_gb[1, 3]],
                             [T_gs[2, 3], T_gb[2, 3]],
                             '--', color="black")
        self.link2 = ax.plot([T_gb[0, 3], T_ge[0, 3]],
                             [T_gb[1, 3], T_ge[1, 3]],
                             [T_gb[2, 3], T_ge[2, 3]],
                             '--', color="black")
        self.link3 = ax.plot([T_ge[0, 3], T_gd[0, 3]],
                             [T_ge[1, 3], T_gd[1, 3]],
                             [T_ge[2, 3], T_gd[2, 3]],
                             '--', color="black")

        # Plot settings
        axis_equal_3dplot(ax)

        return ax


if __name__ == "__main__":
    # Optimized gimbal params
    offset = -pi / 2.0
    roll = 0.0
    pitch = 0.0

    tau_s = np.array([-0.0419, -0.0908, 0.0843, 0.0189, -0.0317, -0.0368])
    tau_d = np.array([0.0032, 0.0007, -0.0302, 1.5800, 0.0123, -1.5695])
    link1 = np.array([offset + roll, 0.0404, -0.0012, 1.5731])
    link2 = np.array([pitch, 0.0, 0.0, 0.0])

    # Gimbal model
    gimbal_model = GimbalModel(
        tau_s=tau_s,
        tau_d=tau_d,
        link1=link1,
        link2=link2
    )

    # Plot gimbal model
    plot = PlotGimbal(gimbal=gimbal_model,
                      show_static_frame=True,
                      show_base_frame=True,
                      show_end_frame=True,
                      show_dynamic_frame=True)
    plot.set_attitude([0.0, 0.0])
    plot.plot()
    plt.show()
