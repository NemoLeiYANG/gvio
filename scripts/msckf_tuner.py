import unittest
import os
import random
import copy
from math import exp


def randval(size):
    base = round(random.uniform(0.0, 9.0), 1)
    exponent = round(random.uniform(0.0, 9.0), 0)

    if size == 1:
        value = base * 10**exponent
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
        "camera.measurement_noise"
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
        elif tweak == "camera.measurement_noise":
            x_tweaked["camera"]["measurement_noise"] = randval(1)
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


def cost_fn(x, iteration, base_dir):
    # import pprint
    # pprint.pprint(x)

    # run_path = os.path.join(base_dir, "run" + str(iteration))
    # os.mkdir(run_path)

    return 0


def simulated_annealing(T, dT, S, tweak_fn, cost_fn):
    # Setup
    best = copy.deepcopy(S)  # initialize best
    best_score = cost_fn(best)

    # Iterate
    iteration = 0
    while T > 0:
        # Tweak current candidate solution
        R = tweak_fn(S)

        # Evaluate
        R_score = cost_fn(R)
        S_score = cost_fn(S)

        # Replace current candidate solution?
        threshold = exp(-(S_score - R_score) / T)
        if R_score < S_score or random.random() < threshold:
            S = copy.deepcopy(R)

        # Keep record of best candidate solution
        if S_score < best_score:
            best = copy.deepcopy(S)
            best_score = S_score

        # Update
        T -= dT  # Decrease temperature
        iteration += 1
        print("iteration: {} \t best_score: {}".format(iteration, best_score))

    return (best, best_score)


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

    def test_cost_fn(self):
        cost_fn(self.x, 0, "/tmp")


if __name__ == "__main__":
    unittest.main()
