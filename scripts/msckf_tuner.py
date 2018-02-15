import unittest
import os
import sys
import time
import copy
import random
from math import exp

import yaml
import numpy as np


def print_usage():
    print("Usage: msckf_tuner.py <config_file>")
    print("Example: msckf_tuner.py ./experiments/configs/kitti_raw_2011_09_26_0005.yaml")


def randval(size):
    base = round(random.uniform(0.0, 9.0), 1)
    exponent = round(random.uniform(0.0, 9.0), 0)

    if size == 1:
        value = base * 10**-exponent
        return value
    else:
        value = []
        for i in range(size):
            value.append(base * 10**-exponent)
        return value


def tweak_fn(x):
    tweak_targets = [
        "max_window_size",
        "max_nb_tracks",
        "min_track_length",
        "imu.initial_covariance.q_init_var",
        "imu.initial_covariance.bg_init_var",
        "imu.initial_covariance.v_init_var",
        "imu.initial_covariance.ba_init_var",
        "imu.initial_covariance.p_init_var",
        "imu.process_noise.w_var",
        "imu.process_noise.dbg_var",
        "imu.process_noise.a_var",
        "imu.process_noise.dba_var",
        "camera.measurement_noise.img_var"
    ]

    nb_tweaks = random.randint(1, 5)
    tweak_targets = random.sample(tweak_targets, nb_tweaks)
    x_tweaked = copy.deepcopy(x)
    for tweak in tweak_targets:
        if tweak == "max_window_size":
            x_tweaked["max_window_size"] = random.randint(30, 100)
        elif tweak == "max_nb_tracks":
            x_tweaked["max_nb_tracks"] = random.randint(30, 200)
        elif tweak == "min_track_length":
            x_tweaked["min_track_length"] = random.randint(5, 20)
        elif tweak == "imu.initial_covariance.q_init_var":
            x_tweaked["imu"]["initial_covariance"]["q_init_var"] = randval(3)
        elif tweak == "imu.initial_covariance.bg_init_var":
            x_tweaked["imu"]["initial_covariance"]["bg_init_var"] = randval(3)
        elif tweak == "imu.initial_covariance.v_init_var":
            x_tweaked["imu"]["initial_covariance"]["v_init_var"] = randval(3)
        elif tweak == "imu.initial_covariance.ba_init_var":
            x_tweaked["imu"]["initial_covariance"]["ba_init_var"] = randval(3)
        elif tweak == "imu.initial_covariance.p_init_var":
            x_tweaked["imu"]["initial_covariance"]["p_init_var"] = randval(3)
        elif tweak == "imu.process_noise.w_var":
            x_tweaked["imu"]["process_noise"]["w_var"] = randval(3)
        elif tweak == "imu.process_noise.dbg_var":
            x_tweaked["imu"]["process_noise"]["dbg_var"] = randval(3)
        elif tweak == "imu.process_noise.a_var":
            x_tweaked["imu"]["process_noise"]["a_var"] = randval(3)
        elif tweak == "imu.process_noise.dba_var":
            x_tweaked["imu"]["process_noise"]["dba_var"] = randval(3)
        elif tweak == "camera.measurement_noise.img_var":
            x_tweaked["camera"]["measurement_noise"]["img_var"] = randval(1)
        else:
            raise RuntimeError("Tweak target [%s] not implemented!" % tweak)

    return x_tweaked


def output_msckf_config(save_path, x):
    config_template = """
# General Settings
max_window_size: {max_window_size}
max_nb_tracks: {max_nb_tracks}
min_track_length: {min_track_length}
enable_ns_trick: true
enable_qr_trick: true

# IMU Settings
imu:
    initial_covariance:
        q_init_var: {q_init_var}
        bg_init_var: {bg_init_var}
        v_init_var: {v_init_var}
        ba_init_var: {ba_init_var}
        p_init_var: {p_init_var}
    process_noise:
        w_var: {w_var}
        dbg_var: {dbg_var}
        a_var: {a_var}
        dba_var: {dba_var}
    constants:
        angular_constant: [0.0, 0.0, 0.0]
        gravity_constant: [0.0, 0.0, -9.8]

# Camera Settings
camera:
    extrinsics:
        p_IC: [0.0, 0.0, 0.0]
        q_CI: [0.5, -0.5, 0.5, -0.5]
    measurement_noise:
        img_var: {img_var}
    """.format(
        max_window_size=x["max_window_size"],
        max_nb_tracks=x["max_nb_tracks"],
        min_track_length=x["min_track_length"],
        q_init_var=x["imu"]["initial_covariance"]["q_init_var"],
        bg_init_var=x["imu"]["initial_covariance"]["bg_init_var"],
        v_init_var=x["imu"]["initial_covariance"]["v_init_var"],
        ba_init_var=x["imu"]["initial_covariance"]["ba_init_var"],
        p_init_var=x["imu"]["initial_covariance"]["p_init_var"],
        w_var=x["imu"]["process_noise"]["w_var"],
        dbg_var=x["imu"]["process_noise"]["dbg_var"],
        a_var=x["imu"]["process_noise"]["a_var"],
        dba_var=x["imu"]["process_noise"]["dba_var"],
        img_var=x["camera"]["measurement_noise"]["img_var"]
    ).lstrip()

    msckf_config = open(save_path, "w")
    msckf_config.write(config_template)
    msckf_config.close()


def parse_data(data_path):
    # Check number of lines
    data_file = open(data_path, "r")
    nb_lines = sum(1 for line in data_file)
    data_file.seek(0)
    if nb_lines <= 2:
        print("No data in [" + data_path + "]?")
        return None

    # Load csv as numpy matrix
    header = data_file.readline()
    data = np.loadtxt(data_file, delimiter=",", skiprows=0)
    data_file.close()

    # Convert numpy matrix as dictionary where each column is represented by
    # its header name
    data_dict = {}
    index = 0
    for element in header.split(","):
        data_dict[element.strip()] = data[:, index]
        index += 1

    return data_dict


def nb_kitti_entries(dataset_path, date, seq):
    seq_path = date + "_drive_" + seq + "_sync"
    data_path = os.path.join(dataset_path, date, seq_path)
    ground_truth_path = os.path.join(data_path, "oxts", "data")

    oxts_files = os.listdir(ground_truth_path)
    return len(oxts_files)


def rms_error(gnd_data, est_data):
    assert gnd_data["t"][0] == 0.0
    assert len(gnd_data["t"]) == len(est_data["t"])

    rms_x = np.sqrt(((est_data["x"] - gnd_data["x"]) ** 2).mean())
    rms_y = np.sqrt(((est_data["y"] - gnd_data["y"]) ** 2).mean())
    rms_z = np.sqrt(((est_data["z"] - gnd_data["z"]) ** 2).mean())
    rms_vx = np.sqrt(((est_data["vx"] - gnd_data["vx"]) ** 2).mean())
    rms_vy = np.sqrt(((est_data["vy"] - gnd_data["vy"]) ** 2).mean())
    rms_vz = np.sqrt(((est_data["vz"] - gnd_data["vz"]) ** 2).mean())
    rms_roll = np.sqrt(((est_data["roll"] - gnd_data["roll"]) ** 2).mean())
    rms_pitch = np.sqrt(((est_data["pitch"] - gnd_data["pitch"]) ** 2).mean())
    rms_yaw = np.sqrt(((est_data["yaw"] - gnd_data["yaw"]) ** 2).mean())

    return (rms_x, rms_y, rms_z,
            rms_vx, rms_vy, rms_vz,
            rms_roll, rms_pitch, rms_yaw)


def cost_fn(x, iteration, config, cache):
    # Check if x has already been evaluated
    if str(x) in cache:
        return cache[str(x)]

    # Output MSCKF config yaml file
    output_path = os.path.join(config["output_dir"], "run" + str(iteration))
    os.system("mkdir -p " + output_path)
    msckf_config_path = os.path.join(output_path, "msckf.yaml")
    output_msckf_config(msckf_config_path, x)

    # Build command line string
    command = "%s %s %s %s %s %s > /dev/null" % (
        config["kitti_runner"],
        config["kitti"]["dataset_path"],
        config["kitti"]["date"],
        config["kitti"]["seq"],
        msckf_config_path,
        output_path
    )
    print(command)

    # Execute MSCKF run and measure time taken
    time_start = time.time()
    os.system(command)
    time_taken = time.time() - time_start

    # Calculate RMS Error
    est_data = parse_data(os.path.join(output_path, "msckf_est.dat"))
    gnd_data = parse_data(os.path.join(output_path, "msckf_gnd.dat"))
    errors = rms_error(gnd_data, est_data)
    total_error = sum(errors)

    # Double check output data
    nb_entries = nb_kitti_entries(config["kitti"]["dataset_path"],
                                  config["kitti"]["date"],
                                  config["kitti"]["seq"])
    cost = float("inf")
    if len(est_data["t"]) != nb_entries:
        cost = float("inf")
    else:
        cost = total_error + time_taken

    # Update cache
    cache[str(x)] = x

    return cost, errors, time_taken


def record_progress(output_path, T, iteration, scores, errors, time_taken):
    # Open results file
    results_file = open(output_path, "a")

    # Write header
    if iteration == 0:
        header = ["#T", "iter", "score_current", "score_best", "score_tweaked",
                  "rms_err_x", "rms_err_y", "rms_err_z",
                  "rms_err_vx", "rms_err_vy", "rms_err_vz",
                  "rms_err_roll", "rms_err_pitch", "rms_err_yaw",
                  "time_taken"]
        results_file.write(",".join(header) + "\n")

    # Record progress
    results_entry = "{0},{1},{2},{3},{4},".format(T, iteration, scores[0],
                                                  scores[1], scores[2])
    results_entry += "{0},{1},{2},".format(errors[0], errors[1], errors[2])
    results_entry += "{0},{1},{2},".format(errors[3], errors[4], errors[5])
    results_entry += "{0},{1},{2},".format(errors[6], errors[7], errors[8])
    results_entry += "{0}".format(time_taken)
    results_entry += "\n"
    results_file.write(results_entry)

    # Close up
    results_file.close()


def simulated_annealing(S, tweak_fn, cost_fn, config):
    # Simulated annealing settings
    T = config["simulated_annealing"]["T"]
    dT = config["simulated_annealing"]["dT"]

    # Cache
    cache = {}

    # Initialize best
    best = copy.deepcopy(S)  # initialize best
    best_score, error, time_taken = cost_fn(best, 0, config, cache)
    S_score = best_score

    # Iterate
    iteration = 0
    time_start = time.time()
    while T > 0:
        # Tweak current candidate solution
        R = tweak_fn(S)

        # Evaluate
        R_score, errors, time_taken = cost_fn(R, iteration, config, cache)

        # Replace current candidate solution?
        threshold = exp((S_score - R_score) / T)
        if S_score > R_score or random.random() < threshold:
            S = copy.deepcopy(R)
            S_score = R_score

        # Keep record of best candidate solution
        if best_score > S_score:
            best = copy.deepcopy(S)
            best_score = S_score

        # Record progress
        record_progress(config["simulated_annealing"]["output_file"],
                        T,
                        iteration,
                        [S_score, best_score, R_score],
                        errors,
                        time_taken)

        # Update
        T -= dT  # Decrease temperature
        iteration += 1
        print("iteration: {} \t best_score: {}".format(iteration,
                                                       round(best_score, 2)))

    # Record best config
    output_msckf_config(config["simulated_annealing"]["best_file"])

    # Print stats
    time_taken = time.time() - time_start
    print("Time taken: {0}".format(time_taken))
    print("Best score: {0}".format(round(best_score, 2)))


class TestSA(unittest.TestCase):
    def setUp(self):
        self.x = {
            "max_window_size": 60,
            "max_nb_tracks": 200,
            "min_track_length": 15,
            "enable_ns_trick": True,
            "enable_qr_trick": True,
            "imu": {
                "initial_covariance": {
                    "q_init_var": [1e-3, 1e-3, 1e-3],
                    "bg_init_var": [1e-3, 1e-3, 1e-3],
                    "v_init_var": [1e-3, 1e-3, 1e-3],
                    "ba_init_var": [1e-3, 1e-3, 1e-3],
                    "p_init_var": [1e-3, 1e-3, 1e-3],
                },
                "process_noise": {
                    "w_var": [1e-2, 1e-2, 1e-2],
                    "dbg_var": [1e-3, 1e-3, 1e-3],
                    "a_var": [1e-2, 1e-2, 1e-2],
                    "dba_var": [1e-3, 1e-3, 1e-3]
                },
                "constants": {
                    "angular_constant": [0.0, 0.0, 0.0],
                    "gravity_constant": [0.0, 0.0, -9.8]
                }
            },
            "camera": {
                "extrinsics": {
                    "p_IC": [0.0, 0.0, 0.0],
                    "q_CI": [0.5, -0.5, 0.5, -0.5]
                },
                "measurement_noise": {
                    "img_var": 1e-2
                }
            }
        }

    def test_randval(self):
        val = randval(3)
        self.assertTrue(len(val), 3)

    def test_tweak_fn(self):
        x_tweaked = tweak_fn(self.x)
        for i in range(100):
            self.assertNotEqual(str(x_tweaked), str(self.x))

    def test_rms_error(self):
        est_data = parse_data("/tmp/msckf/msckf_est.dat")
        gnd_data = parse_data("/tmp/msckf/msckf_gnd.dat")
        errors = rms_error(gnd_data, est_data)
        self.assertTrue(len(errors) == 9)

    def test_nb_kitti_entries(self):
        nb_kitti_entries("/data/kitti/raw", "2011_09_26", "0005")

    def test_cost_fn(self):
        config = {
            "output_dir": "/tmp/experiment",
            "kitti_runner": "./build/experiments/kitti_runner",
            "kitti": {
                "dataset_path": "/data/kitti/raw",
                "date": "2011_09_26",
                "seq": "0005"
            }
        }
        cache = {}
        print(cost_fn(self.x, 0, config, cache))


if __name__ == "__main__":
    # Check CLI args
    if len(sys.argv) != 2:
        print_usage()

    # Load config
    config_path = sys.argv[1]
    config_file = open(config_path, "r")
    config = yaml.load(config_file)

    # Initial config
    x = {
        "max_window_size": 60,
        "max_nb_tracks": 200,
        "min_track_length": 15,
        "enable_ns_trick": True,
        "enable_qr_trick": True,
        "imu": {
            "initial_covariance": {
                "q_init_var": [1e-3, 1e-3, 1e-3],
                "bg_init_var": [1e-3, 1e-3, 1e-3],
                "v_init_var": [1e-3, 1e-3, 1e-3],
                "ba_init_var": [1e-3, 1e-3, 1e-3],
                "p_init_var": [1e-3, 1e-3, 1e-3],
            },
            "process_noise": {
                "w_var": [1e-2, 1e-2, 1e-2],
                "dbg_var": [1e-3, 1e-3, 1e-3],
                "a_var": [1e-2, 1e-2, 1e-2],
                "dba_var": [1e-3, 1e-3, 1e-3]
            },
            "constants": {
                "angular_constant": [0.0, 0.0, 0.0],
                "gravity_constant": [0.0, 0.0, -9.8]
            }
        },
        "camera": {
            "extrinsics": {
                "p_IC": [0.0, 0.0, 0.0],
                "q_CI": [0.5, -0.5, 0.5, -0.5]
            },
            "measurement_noise": {
                "img_var": 1e-2
            }
        }
    }

    # Perform MSCKF Tuning
    simulated_annealing(x, tweak_fn, cost_fn, config)
