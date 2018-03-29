import argparse
from collections import OrderedDict

import rosbag
import numpy as np
from scipy.optimize import nnls


def allan_variance(x, dt=1, min_cluster_size=1, min_cluster_count='auto',
                   n_clusters=100, input_type="increment"):
    """Compute Allan variance (AVAR).

    Consider an underlying measurement y(t). Our sensors output integrals of
    y(t) over successive time intervals of length dt. These measurements
    x(k * dt) form the input to this function.

    Allan variance is defined for different averaging times tau = m * dt as
    follows::

        AVAR(tau) = 1/2 * <(Y(k + m) - Y(k))>,

    where Y(j) is the time average value of y(t) over [k * dt, (k + m) * dt]
    (call it a cluster), and < ... > means averaging over different clusters.
    If we define X(j) being an integral of x(s) from 0 to dt * j,
    we can rewrite the AVAR as  follows::

        AVAR(tau) = 1/(2 * tau**2) * <X(k + 2 * m) - 2 * X(k + m) + X(k)>

    We implement < ... > by averaging over different clusters of a given sample
    with overlapping, and X(j) is readily available from x.

    Parameters
    ----------
    x : ndarray, shape (n, ...)
        Integrating sensor readings, i. e. its cumulative sum gives an
        integral of a signal. Assumed to vary along the 0-th axis.
    dt : float, optional
        Sampling period. Default is 1.
    min_cluster_size : int, optional
        Minimum size of a cluster to use. Determines a lower bound on the
        averaging time as ``dt * min_cluster_size``. Default is 1.
    min_cluster_count : int or 'auto', optional
        Minimum number of clusters required to compute the average. Determines
        an upper bound of the averaging time as
        ``dt * (n - min_cluster_count) // 2``. If 'auto' (default) it is taken
        to be ``min(1000, n - 2)``
    n_clusters : int, optional
        Number of clusters to compute Allan variance for. The averaging times
        will be spread approximately uniform in a log scale. Default is 100.
    input_type : 'increment' or 'mean', optional
        How to interpret the input data. If 'increment' (default), then
        `x` is assumed to contain integral increments over successive time
        intervals (as described above). If 'mean', then `x` is assumed to
        contain mean values of y over successive time intervals, i.e.
        increments divided by `dt`.

    Returns
    -------
    tau : ndarray
        Averaging times for which Allan variance was computed, 1-d array.
    avar : ndarray
        Values of AVAR. The 0-th dimension is the same as for `tau`. The
        trailing dimensions match ones for `x`.

    References
    ----------
    .. [1] https://en.wikipedia.org/wiki/Allan_variance
    """
    if input_type not in ('increment', 'mean'):
        raise ValueError("`input_type` must be either 'increment' or 'mean'.")

    x = np.asarray(x, dtype=float)
    n = x.shape[0]
    X = np.cumsum(x, axis=0)

    if min_cluster_count == 'auto':
        min_cluster_count = min(1000, n - 2)

    log_min = np.log2(min_cluster_size)
    log_max = np.log2((n - min_cluster_count) // 2)

    cluster_sizes = np.logspace(log_min, log_max, n_clusters, base=2)
    cluster_sizes = np.unique(np.round(cluster_sizes)).astype(np.int64)

    avar = np.empty(cluster_sizes.shape + X.shape[1:])
    for i, k in enumerate(cluster_sizes):
        c = X[2*k:] - 2 * X[k:-k] + X[:-2*k]
        avar[i] = np.mean(c**2, axis=0) / k / k

    if input_type == 'increment':
        avar *= 0.5 / dt**2
    elif input_type == 'mean':
        avar *= 0.5

    return cluster_sizes * dt, avar


def params_from_avar(tau, avar, output_type='array'):
    """Estimate noise parameters from Allan variance.

    The parameters being estimated are typical for inertial sensors:
    quantization noise, additive white noise, flicker noise (long term bias
    instability), random walk and linear ramp (this is a deterministic effect).

    The parameters are estimated using linear least squares with weights
    inversely proportional to the values of Allan variance. That is the sum of
    relative error is minimized. This approach is approximately equivalent of
    doing estimation in the log-log scale.

    Parameters
    ----------
    tau : ndarray, shape (n,)
        Values of averaging time.
    avar : ndarray, shape (n,)
        Values of Allan variance corresponding to `tau`.
    output_type : 'array' or 'dict', optional
        How to return the computed parameters. If 'array' (default), then
        ndarray is returned. If 'dict', then OrderedDict is returned. See
        Returns section for more details.

    Returns
    -------
    params : ndarray, shape (5,) or OrderedDict
        Either ndarray with estimated parameters, ordered as quantization,
        additive white, flicker, random walk, linear ramp. Or OrderedDict with
        the keys 'quantization', 'white', 'flicker', 'walk', 'ramp' and the
        estimated parameters as the values.
    prediction : ndarray, shape (n,)
        Predicted values of Allan variance using the estimated parameters.
    """
    if output_type not in ('array', 'dict'):
        raise ValueError("`output_type` must be either 'array' or 'dict'.")

    n = tau.shape[0]
    A = np.empty((n, 5))
    A[:, 0] = 3 / tau**2
    A[:, 1] = 1 / tau
    A[:, 2] = 2 * np.log(2) / np.pi
    A[:, 3] = tau / 3
    A[:, 4] = tau**2 / 2
    A /= avar[:, np.newaxis]
    b = np.ones(n)

    x = nnls(A, b)[0]
    prediction = A.dot(x) * avar
    params = np.sqrt(x)

    if output_type == 'dict':
        params = OrderedDict(
            zip(('quantization', 'white', 'flicker', 'walk', 'ramp'), params))

    return params, prediction


def parse_bag(bag, imu_topic):
    """Parse ROS bag

    Parameters
    ----------
    bag : rosbag
        ROS bag containing IMU data
    imu_topic : str
        IMU topic

    Returns
    -------
    imu_data : dict
        Dictionary containing accelerometer and gyroscope data
    dt : float
        Time difference

    """
    dt = 1.0 / info.topics[imu_topic].frequency
    imu_data = {"accel": [], "gyro": []}
    for topic, msg, t in bag.read_messages(topics=[imu_topic]):
        imu_data["accel"].append([msg.linear_acceleration.x,
                                  msg.linear_acceleration.y,
                                  msg.linear_acceleration.z])
        imu_data["gyro"].append([msg.angular_velocity.x,
                                 msg.angular_velocity.y,
                                 msg.angular_velocity.z])
    imu_data["gyro"] = np.array(imu_data["gyro"])
    imu_data["accel"] = np.array(imu_data["accel"])

    return imu_data, dt


def analyze_imu_noise(imu_data, dt):
    """Analyze IMU noise

    Parameters
    ----------
    imu_data: dict
        Dictionary containing IMU data
    dt : float
        Sample time

    Returns
    -------
    params : tuple
        Containing Allan Variance noise characteristics
    tau : tuple of ndarray
        Averaging times for which Allan variance was computed, 1-d array.
    avar : tuple of ndarray
        Values of AVAR. The 0-th dimension is the same as for `tau`. The
        trailing dimensions match ones for `x`.

    """
    # Angular velocity Allan Variance Analysis
    gx_tau, gx_av = allan_variance(imu_data["gyro"][:, 0], dt)
    gy_tau, gy_av = allan_variance(imu_data["gyro"][:, 1], dt)
    gz_tau, gz_av = allan_variance(imu_data["gyro"][:, 2], dt)
    gx_params, av_pred = params_from_avar(gx_tau, gx_av, output_type='dict')
    gy_params, av_pred = params_from_avar(gy_tau, gy_av, output_type='dict')
    gz_params, av_pred = params_from_avar(gz_tau, gz_av, output_type='dict')

    # Acceleration Allan Variance Analysis
    ax_tau, ax_av = allan_variance(imu_data["accel"][:, 0], dt)
    ay_tau, ay_av = allan_variance(imu_data["accel"][:, 1], dt)
    az_tau, az_av = allan_variance(imu_data["accel"][:, 2], dt)
    ax_params, av_pred = params_from_avar(ax_tau, ax_av, output_type='dict')
    ay_params, av_pred = params_from_avar(ay_tau, ay_av, output_type='dict')
    az_params, av_pred = params_from_avar(az_tau, az_av, output_type='dict')

    params = (gx_params, gy_params, gz_params, ax_params, ay_params, az_params)
    taus = (gx_tau, gy_tau, gz_tau, ax_tau, ay_tau, az_tau)
    av = (gx_av, gy_av, gz_av, ax_av, ay_av, az_av)

    return params, taus, av


def print_params(target, axis, params):
    """Prints the noise characteristics on screen

    Parameters
    ----------
    target : str
        Gyroscope or Accelerometer
    axis : str
        x, y or z-axis
    params : dict
        Dictionary containing IMU noise characteristics

    """
    print("%s - %s" % (target, axis))
    print("quatization: %f" % params["quantization"])
    print("white noise: %f" % params["white"])
    print("flicker: %f" % params["flicker"])
    print("random walk: %f" % params["walk"])
    print("ramp: %f" % params["ramp"])
    print("")


def output_results(results_file, target, axis, params):
    """Output IMU Allan Variance results to file

    Parameters
    ----------
    target : str
        Gyroscope or Accelerometer
    axis : str
        x, y or z-axis
    params : dict
        Dictionary containing IMU noise characteristics

    """
    results_file.write("%s - %s\n" % (target, axis))
    results_file.write("quatization: %f\n" % params["quantization"])
    results_file.write("white noise: %f\n" % params["white"])
    results_file.write("flicker: %f\n" % params["flicker"])
    results_file.write("random walk: %f\n" % params["walk"])
    results_file.write("ramp: %f\n" % params["ramp"])
    results_file.write("\n")


def output_kalibr_imu_config(imu_params, imu_topic, imu_rate):
    gx_params, gy_params, gz_params = imu_params[0:3]
    ax_params, ay_params, az_params = imu_params[3:6]

    accel_noise_density = max(ax_params["white"],
                              ay_params["white"],
                              az_params["white"])
    accel_random_walk = max(ax_params["walk"],
                            ay_params["walk"],
                            az_params["walk"])
    gyro_noise_density = max(gx_params["white"],
                             gy_params["white"],
                             gz_params["white"])
    gyro_random_walk = max(gx_params["walk"],
                           gy_params["walk"],
                           gz_params["walk"])

    imu_config = """\
# Accelerometers
accelerometer_noise_density: {accel_noise_density}  # Noise density (continuous-time)
accelerometer_random_walk: {accel_random_walk}  # Bias random walk

# Gyroscopes
gyroscope_noise_density: {gyro_noise_density}  # Noise density (continuous-time)
gyroscope_random_walk: {gyro_random_walk}  # Bias random walk

rostopic: {imu_topic}  # the IMU ROS topic
update_rate: {imu_rate}  # Hz (for discretization of the values above)
    """.format(
        accel_noise_density=accel_noise_density,
        accel_random_walk=accel_random_walk,
        gyro_noise_density=gyro_noise_density,
        gyro_random_walk=gyro_random_walk,
        imu_topic=imu_topic,
        imu_rate=imu_rate
    )
    imu_config_file = open("imu.yaml", "w")
    imu_config_file.write(imu_config.strip())
    imu_config_file.close()


if __name__ == "__main__":
    # Parse CLI args
    description = 'Find IMU noise characteristics via Allan Variance'
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--bag', help='ROS bag', required=True)
    parser.add_argument('--imu_topic', help='IMU data topic', required=True)
    args = parser.parse_args()

    # Setup results file
    results_file = open("results.txt", "w")

    # ROS bag
    bag_path = args.bag
    imu_topic = args.imu_topic
    print("Opening ROS bag [%s] ..." % bag_path)
    bag = rosbag.Bag(bag_path, 'r')

    # Check if topic is in bag
    info = bag.get_type_and_topic_info()
    if imu_topic not in info.topics:
        raise RuntimeError("Opps! topic not in bag!")

    # Parse ROS bag
    print("Parsing ROS bag ...")
    imu_data, dt = parse_bag(bag, imu_topic)

    # Analyze IMU data
    print("Analyzing data ...")
    params, taus, av = analyze_imu_noise(imu_data, dt)

    # Output results to screen
    print("\nResults")
    print("-" * 60)
    print_params("Gyroscope", "x-axis", params[0])
    print_params("Gyroscope", "y-axis", params[1])
    print_params("Gyroscope", "z-axis", params[2])
    print_params("Accelerometer", "x-axis", params[3])
    print_params("Accelerometer", "y-axis", params[4])
    print_params("Accelerometer", "z-axis", params[5])

    # Output results to file
    output_results(results_file, "Gyroscope", "x-axis", params[0])
    output_results(results_file, "Gyroscope", "y-axis", params[1])
    output_results(results_file, "Gyroscope", "z-axis", params[2])
    output_results(results_file, "Accelerometer", "x-axis", params[3])
    output_results(results_file, "Accelerometer", "y-axis", params[4])
    output_results(results_file, "Accelerometer", "z-axis", params[5])
    results_file.close()

    # Output Kalibr IMU yaml file
    imu_rate = 1.0 / dt
    output_kalibr_imu_config(params, imu_topic, imu_rate)
