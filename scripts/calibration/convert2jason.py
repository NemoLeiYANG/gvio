#!/usr/bin/env python
"""
This script takes the output of aprilgrid_extractor.py and converts it into a format that
encoderless_gimbal_calib recognizes.

The output of this script is:

- Gimbal and static camera measurement files, such as `0_gimbal.txt` or
`0_static.txt`, where the files are prefixed with the n-th measurement. Each of
these files has the format:

    gridpoints:
    <AprilGrid 3d object point> <2D correspondances>

    tmatrix:
    <SolvePnP result of AprilGrid object points and 2d correspondance
    as a 4x4 transform>

    gimbalangles:
    <Gimbal joint angles>
    end:

Example:

    gridpoints:
    0.57200 0.00000 0.00000 566.34528 470.93115
    0.66000 0.00000 0.00000 610.41833 458.62225
    ...
    0.45760 0.66000 0.00000 481.11957 66.62180
    0.66000 0.66000 0.00000 588.97180 74.37969
    tmatrix:
    0.99454 -0.03362 -0.09874 -0.25430
    -0.04560 -0.99152 -0.12169 0.39716
    -0.09382 0.12553 -0.98764 0.52150
    0.00000 0.00000 0.00000 1.00000
    gimbalangles:
    -0.399985 -0.199801
    end:

- initialGuess.txt, contains the initial guess of the gimbal params and has the
following format:

    % All parameters and whether to use the parameter in optimization (1 or 0)
    start:
    6dof, <roll pitch yaw x y z> 1 1 1 1 1 1
    2
    dh, <0 d a alpha> -90 1 1 1 1 -20 20
    dh, <0 d a alpha> 0 1 0 0 0 -20 20
    6dof, <roll pitch yaw x y z> 1 1 1 1 1 1
    end:

Example:

    % All parameters and whether to use the parameter in optimization (1 or 0)
    start:
    6dof, 0 0 0 -0.045 -0.085 0.80 1 1 1 1 1 1
    2
    dh, 0 0.45 0 90 -90 1 1 1 1 -20 20
    dh, 0 0.0001 0 0 0 1 0 0 0 -20 20
    6dof, 90 0 -90 0.01 0 -0.03 1 1 1 1 1 1
    end:

- cameraParams_equi.txt, contains camera intrinsics and distortion coefficients

    % Camera Intrinsics format: name [1], resolution[2], distortions[4] [k1 k2 k3 k4], intrinsic matrix[3x3]
    start:
    pinhole
    equi
    2
    static, {image_width} {image_height}, {static_distortions}, {static_intrinsics}
    gimbal, {image_width} {image_height}, {gimbal_distortions}, {gimbal_intrinsics}
    end:

Example:

    % Camera Intrinsics format: name [1], resolution[2], distortions[4] [k1 k2 k3 k4], intrinsic matrix[3x3]
    start:
    pinhole
    equi
    2
    static, 752, 480, -0.0643644949705 0.00698652279462 -0.00868155218634 0.00277718269368, 391.173204324 0.0 369.907286564 0.0 393.029045377 239.541787098 0.0 0.0 1.0
    gimbal, 752, 480, -0.12020923706 0.0814633228588 -0.131458378902 0.0715992059013, 522.797647275 0.0 365.303720691 0.0 524.702078608 237.815980904 0.0 0.0 1.0
    end:

- transforms.txt, some kind of transforms file but it is not important


- targetParams.txt, target parameter file but it is not used


"""
import re
import os
from os import listdir
from os.path import join
import shutil
import sys
import argparse

import yaml


def atoi(text):
    return int(text) if text.isdigit() else text.lower()


def natural_keys(text):
    '''
    alist.sort(key=natural_keys) sorts in human order
    http://nedbatchelder.com/blog/200712/human_sorting.html
    (See Toothy's implementation in the comments)
    '''
    return [atoi(c) for c in re.split('(\d+)', text)]


def camera_intrinsics(camchain, key):
    fx = camchain[key]["intrinsics"][0]
    fy = camchain[key]["intrinsics"][1]
    cx = camchain[key]["intrinsics"][2]
    cy = camchain[key]["intrinsics"][3]

    K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    K_str = " ".join(map(str, K))

    return K_str


def output_camera_params_file(camchain_file, output_path):
    stream = open(camchain_file, "r")
    camchain = yaml.load(stream)

    static_cam_distortion = " ".join(map(str, camchain["cam0"]["distortion_coeffs"]))
    gimbal_cam_distortion = " ".join(map(str, camchain["cam2"]["distortion_coeffs"]))

    static_cam_intrinsics = camera_intrinsics(camchain, "cam0")
    gimbal_cam_intrinsics = camera_intrinsics(camchain, "cam2")

    params = """\
% Camera Intrinsics format: name [1], resolution[2], distortions[4] [k1 k2 k3 k4], intrinsic matrix[3x3]
start:
pinhole
equi
2
static, {image_width} {image_height}, {static_distortions}, {static_intrinsics}
gimbal, {image_width} {image_height}, {gimbal_distortions}, {gimbal_intrinsics}
end:
    """.format(image_width=camchain["cam0"]["resolution"][0],
               image_height=camchain["cam0"]["resolution"][1],
               static_distortions=static_cam_distortion,
               gimbal_distortions=gimbal_cam_distortion,
               static_intrinsics=static_cam_intrinsics,
               gimbal_intrinsics=gimbal_cam_intrinsics).strip()

    params_file = open(os.path.join(output_path, "cameraParams_equi.txt"), "w")
    params_file.write(params)
    params_file.close()


def output_initial_guess(output_path):
    # IMPORTANT NOTE: THIS INITIAL GUESS IS FOR THE TRICLOPS ONLY!
    output = """\
% All parameters and whether to use the parameter in optimization (1 or 0)
start:
6dof, 0 0 0 -0.045 -0.085 0.80 1 1 1 1 1 1
2
dh, 0 0.45 0 90 -90 1 1 1 1 -20 20
dh, 0 0.0001 0 0 0 1 0 0 0 -20 20
6dof, 90 0 -90 0.01 0 -0.03 1 1 1 1 1 1
end:
    """.strip()
    output_file = open(os.path.join(output_path, "initialGuess.txt"), "w")
    output_file.write(output)
    output_file.close()


def output_transforms_file(output_path):
    output = """\
% Transforms for simulated measurements
% Origin World
world:
1 0 0 0
0 1 0 0
0 0 1 0
0 0 0 1

% Transform from target (chessboard) to world
w_T_cb:
0 0 -1 2
0 -1 0 0.10
-1 0 0 0.10
0 0 0 1

% Transform from Base of Mechanism to world
w_T_base:
0 0 1 0
-1 0 0 0
0 -1 0 0
0 0 0 1
    """.strip()
    output_file = open(os.path.join(output_path, "transforms.txt"), "w")
    output_file.write(output)
    output_file.close()


def output_target_file(output_path):
    output = """\
% Target parameters (Assume chessboard)
sqSize:
0.06
width:
6
height
6
    """.strip()
    output_file = open(os.path.join(output_path, "targetParams.txt"), "w")
    output_file.write(output)
    output_file.close()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument('--data', type=str, help='Path to data', required=True)
    p.add_argument('--target', type=str, help='Target file', required=True)
    p.add_argument('--camchain', type=str, help='Camchain file', required=True)
    p.add_argument('--output', type=str, help='Output path', required=True)
    args = p.parse_args()

    # Form static camera and gimbal camera paths
    static_cam_path = os.path.join(args.data, "cam0", "points_data")
    gimbal_cam_path = os.path.join(args.data, "cam2", "points_data")

    # Check if static cam or gimbal cam path does not exist
    if os.path.isdir(static_cam_path) is False:
        raise RuntimeError("Dir [%s] does not exist!" % static_cam_path)
    if os.path.isdir(gimbal_cam_path) is False:
        raise RuntimeError("Dir [%s] does not exist!" % gimbal_cam_path)

    # Get points data
    static_files = [join(static_cam_path, f) for f in listdir(static_cam_path)]
    static_files = sorted(static_files, key=natural_keys)
    gimbal_files = [join(gimbal_cam_path, f) for f in listdir(gimbal_cam_path)]
    gimbal_files = sorted(gimbal_files, key=natural_keys)

    # Create output dir
    if os.path.isdir(args.output) is False:
        os.mkdir(args.output)

    # Output files
    output_camera_params_file(args.camchain, args.output)
    output_initial_guess(args.output)
    output_transforms_file(args.output)
    output_target_file(args.output)

    # Copy file over
    nb_files = len(static_files)
    for i in range(nb_files):
        # Static file
        static_file = static_files[i]
        static_output_file = os.path.basename(static_file)
        static_output_file = static_output_file.replace(".txt", "_static.txt")
        static_output_file = os.path.join(args.output, static_output_file)
        shutil.copyfile(static_file, static_output_file)

        # Gimbal file
        gimbal_file = gimbal_files[i]
        gimbal_output_file = os.path.basename(gimbal_file)
        gimbal_output_file = gimbal_output_file.replace(".txt", "_gimbal.txt")
        gimbal_output_file = os.path.join(args.output, gimbal_output_file)
        shutil.copyfile(gimbal_file, gimbal_output_file)
