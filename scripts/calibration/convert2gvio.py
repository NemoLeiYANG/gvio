#!/usr/bin/env python2
"""
This script takes the camchain file from Kalibr and optimized gimbal params
from encoderless gimbal params to generate a GVIO compatible camchain.yaml

The optimized params from encoderless_gimbal_calib is stored as a matlab matrix
to GVIO's yaml format.

optimized_params.mat is a 17x1 vector:

- tau_s, first six numbers representing (roll, pitch, yaw, x, y, z)
- link 1 containing 4 DH params (theta, d, a, alpha)
- link 2 but only represented by 1 meaningless 0
- tau_d, last six numbers representing (roll, pitch, yaw, x, y, z)

6 + 4 + 1 + 6 = 17

GVIO's camchain file format is as follows:

    cam0:
      camera_model: <str>
      distortion_coeffs: <vector>
      distortion_model: <str>
      intrinsics: <vector>
      resolution: <vector>
      rostopic: <str>

    cam1:
      camera_model: <str>
      distortion_coeffs: <vector>
      distortion_model: <str>
      intrinsics: <vector>
      resolution: <vector>
      rostopic: <str>

    cam2:
      camera_model: <str>
      distortion_coeffs: <vector>
      distortion_model: <str>
      intrinsics: <vector>
      resolution: <vector>
      rostopic: <str>

    T_C1_C0:
      rows: int
      cols: int
      data : <vector>

    T_C2_C0:
      tau_s: [x, y, z, roll, pitch, yaw]
      tau_d: [x, y, z, roll, pitch, yaw]
      w1: [d, a, alpha]
      w2: [d, a, alpha]
      theta1_offset: -1.5707
      theta2_offset: 0.0

"""
import argparse

import yaml
import scipy.io
import numpy as np
from numpy import array2string as a2s


def parse_kalibr_camchain(camchain_file):
    stream = open(camchain_file, "r")
    camchain = yaml.load(stream)

    camera_params = []
    camera_params.append(camchain["cam0"])
    camera_params.append(camchain["cam1"])
    camera_params.append(camchain["cam2"])

    return camera_params


def parse_gimbal_optimized_params(params_file):
    """ `optimized_params.mat` is a 17x1 vector:

    - tau_s, first six numbers representing (roll, pitch, yaw, x, y, z)
    - link 1 containing 4 DH params (theta, d, a, alpha)
    - link 2 but only represented by 1 meaningless 0
    - tau_d, last six numbers representing (roll, pitch, yaw, x, y, z)

    6 + 4 + 1 + 6 = 17

    """
    mat = scipy.io.loadmat(params_file)
    params = mat["optimized_params"].flatten()
    tau_s = np.array([params[3], params[4], params[5],   # x, y, z
                      params[0], params[1], params[2]])  # roll, pitch yaw
    w1 = np.array([params[7], params[8], params[9]])
    w2 = np.array([0, 0, 0])
    tau_d = np.array([params[14], params[15], params[16],   # x, y, z
                      params[11], params[12], params[13]])  # roll, pitch yaw

    gimbal_params = {"tau_s": tau_s, "w1": w1, "w2": w2, "tau_d": tau_d}
    return gimbal_params


def generate_gvio_camchain(camera_params, gimbal_params, output_path):
    camchain = ""

    # Write cameras
    camera_index = 0
    for cam in camera_params:
        camera_property = """\
cam{camera_index}:
  camera_model: {camera_model}
  distortion_coeffs: {distortion_coeffs}
  distortion_model: {distortion_model}
  intrinsics: {intrinsics}
  resolution: {resolution}
  rostopic: {rostopic}
        """.format(camera_index=camera_index,
                   camera_model=cam["camera_model"],
                   distortion_coeffs=cam["distortion_coeffs"],
                   distortion_model=cam["distortion_model"],
                   intrinsics=cam["intrinsics"],
                   resolution=cam["resolution"],
                   rostopic=cam["rostopic"])
        camera_index += 1
        camchain += camera_property
        camchain += "\n"

    # Write transform from cam0 to cam1 (a.k.a. camera extrinsics)
    T_C1_C0 = camera_params[1]["T_cn_cnm1"]
    camchain += """\
T_C1_C0:
  rows: 4
  cols: 4
  data: [
    {R00}, {R01}, {R02}, {tx},
    {R10}, {R11}, {R12}, {ty},
    {R20}, {R21}, {R22}, {tz},
    0.0, 0.0, 0.0, 1.0
  ]
    """.format(R00=T_C1_C0[0][0], R01=T_C1_C0[0][1], R02=T_C1_C0[0][2],
               R10=T_C1_C0[1][0], R11=T_C1_C0[1][1], R12=T_C1_C0[1][2],
               R20=T_C1_C0[2][0], R21=T_C1_C0[2][1], R22=T_C1_C0[2][2],
               tx=T_C1_C0[0][3], ty=T_C1_C0[1][3], tz=T_C1_C0[2][3]).strip()
    camchain += "\n"
    camchain += "\n"

    # Write gimbal transform from cam0 to cam2 (gimbal camera)
    camchain += """\
T_C2_C0:
  tau_s: {tau_s}
  tau_d: {tau_d}
  w1: {w1}
  w2: {w2}
  theta1_offset: -1.5707
  theta2_offset: 0.0
    """.format(
        tau_s=a2s(gimbal_params["tau_s"], separator=",", max_line_width=500),
        tau_d=a2s(gimbal_params["tau_d"], separator=",", max_line_width=500),
        w1=a2s(gimbal_params["w1"], separator=","),
        w2=a2s(gimbal_params["w2"], separator=",")
    )
    camchain += "\n"

    # output GVIO camchain file
    output_file = open(output_path, "w")
    output_file.write(camchain)
    output_file.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Create camchain for GVIO')
    parser.add_argument('--camchain',
                        required=True,
                        type=str,
                        help="camchain file produced by Kalibr")
    parser.add_argument('--optimized_params',
                        required=True,
                        type=str,
                        help="Params produced by encoderless_gimbal_calib")
    parser.add_argument('--output',
                        required=True,
                        type=str,
                        help="Path to output gvio camchain file")
    args = parser.parse_args()

    # Parse Kalibr camchain file
    camera_params = parse_kalibr_camchain(args.camchain)

    # Parse encoderless_gimbal_calib optimized gimbal params
    gimbal_params = parse_gimbal_optimized_params(args.optimized_params)

    # Generate GVIO camchain file
    generate_gvio_camchain(camera_params, gimbal_params, args.output)
