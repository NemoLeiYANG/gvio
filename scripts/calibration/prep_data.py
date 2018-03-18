import os
from os.path import basename
from os.path import splitext
import glob
import argparse

import numpy as np


def mkdir(path):
    if os.path.exists(path) is False:
        os.mkdir(path)


def walkdir(path, ext=None):
    """Walk directory
    Parameters
    ----------
    path : str
        Path to walk
    ext : str
        Filter file extensions (Default value = None)
    Returns
    -------
        List of files
    """
    files = []
    for (dirpath, dirnames, filenames) in os.walk(path):
        for filename in filenames:
            if ext is not None and filename.endswith(ext):
                files.append(os.sep.join([dirpath, filename]))
            elif ext is None:
                files.append(os.sep.join([dirpath, filename]))

    files.reverse()
    return files


class Measurement:
    def __init__(self, filepath):
        self.data = self.load(filepath)

    def parse_gridpoints_line(self, line, data):
        # Parse line
        elements = line.strip().split(" ")
        elements = [float(x) for x in elements]
        x, y, z = elements[0:3]
        u, v = elements[3:5]

        # Form point 3d and 2d
        point3d = [x, y, z]
        point2d = [u, v]

        # Add to storage
        data["target_points"].append(point3d)
        data["corners3d"].append(point3d)
        data["corners2d"].append(point2d)

    def parse_transform(self, line, data):
        # Parse transform
        elements = line.strip().split(" ")
        elements = [float(x) for x in elements]
        data["T_c_t"] += elements

    def parse_joint_angles(self, line, data):
        # Parse gimbal angles
        elements = line.strip().split(" ")
        data["joint_angles"] += [float(x) for x in elements]

    def transform_corners(self, data):
        data["T_c_t"] = np.array(data["T_c_t"]).reshape((4, 4))
        data["corners3d"] = np.array(data["corners3d"])
        data["corners2d"] = np.array(data["corners2d"])

        # Transform the 3d points
        # -- Convert 3d points to homogeneous coordinates
        nb_corners = data["corners3d"].shape[0]
        ones = np.ones((nb_corners, 1))
        corners_homo = np.block([data["corners3d"], ones])
        corners_homo = corners_homo.T
        # -- Transform 3d points
        X = np.dot(data["T_c_t"], corners_homo)
        X = X.T
        data["corners3d"] = X[:, 0:3]

    def load(self, filepath):
        # Setup
        datafile = open(filepath, "r")
        mode = None

        # Data
        data = {
            "target_points": [],
            "corners3d": [],
            "corners2d": [],
            "joint_angles": [],
            "T_c_t": []  # Transform, target to camera
        }

        # Parse file
        for line in datafile:
            line = line.strip()

            if line == "gridpoints:":
                mode = "gridpoints"
            elif line == "tmatrix:":
                mode = "tmatrix"
            elif line == "gimbalangles:":
                mode = "gimbalangles"
            elif line == "end:":
                mode = None
            else:
                if mode == "gridpoints":
                    self.parse_gridpoints_line(line, data)
                elif mode == "tmatrix":
                    self.parse_transform(line, data)
                elif mode == "gimbalangles":
                    self.parse_joint_angles(line, data)

        # Finish up
        self.transform_corners(data)
        data["target_points"] = np.array(data["target_points"])
        datafile.close()

        return data


def filter_common_observations(static, gimbal):
    static_idx = 0
    gimbal_idx = 0

    P_s = []
    P_d = []
    Q_s = []
    Q_d = []

    # Find common target points and store the
    # respective points in 3d and 2d
    for pt_a in static.data["target_points"]:
        for pt_b in gimbal.data["target_points"]:
            if np.array_equal(pt_a, pt_b):
                # Corners 3d observed in both the static and dynamic cam
                P_s.append(static.data["corners3d"][static_idx])
                P_d.append(gimbal.data["corners3d"][gimbal_idx])
                # Corners 2d observed in both the static and dynamic cam
                Q_s.append(static.data["corners2d"][static_idx])
                Q_d.append(gimbal.data["corners2d"][gimbal_idx])
                break

            else:
                gimbal_idx += 1

        static_idx += 1
        gimbal_idx = 0

    P_s = np.array(P_s)
    P_d = np.array(P_d)
    Q_s = np.array(Q_s)
    Q_d = np.array(Q_d)

    return [P_s, P_d, Q_s, Q_d]


if __name__ == "__main__":
    # Parse CLI args
    p = argparse.ArgumentParser(description='Preprocess AprilGrid data')
    p.add_argument('--data', help='data path', required=True)
    p.add_argument('--output', help='output path', required=True)
    args = p.parse_args()

    # Get files
    static_files = glob.glob(os.path.join(args.data, "*_static.txt"))
    gimbal_files = glob.glob(os.path.join(args.data, "*_gimbal.txt"))

    # Sort files
    static_files.sort(key=lambda f: int(basename(f).split("_")[0]))
    gimbal_files.sort(key=lambda f: int(basename(f)[0].split("_")[0]))

    # Check number of images
    length = len(static_files)
    for files in [static_files, gimbal_files]:
        if len(files) != length:
            raise RuntimeError("Unequal number of image files")

    # Create measurement sets
    nb_measurements = length
    output_dir = args.output
    mkdir(output_dir)
    joint_file = open(os.path.join(output_dir, "joint.csv"), "w")

    for i in range(nb_measurements):
        # Load i-th measurement from each camera
        static_mea = Measurement(static_files[i])
        gimbal_mea = Measurement(gimbal_files[i])

        # Record gimbal angles
        joint_angles = static_mea.data["joint_angles"]
        joint_file.write(",".join(str(x) for x in joint_angles) + "\n")

        # Find common points between both cameras
        Z_i = filter_common_observations(static_mea, gimbal_mea)
        P_s, P_d, Q_s, Q_d = Z_i

        # Output measurement set
        output_path = os.path.join(output_dir, "set%d" % i)
        mkdir(output_path)
        np.savetxt(os.path.join(output_path, "P_s"), P_s, delimiter=",")
        np.savetxt(os.path.join(output_path, "P_d"), P_d, delimiter=",")
        np.savetxt(os.path.join(output_path, "Q_s"), Q_s, delimiter=",")
        np.savetxt(os.path.join(output_path, "Q_d"), Q_d, delimiter=",")
