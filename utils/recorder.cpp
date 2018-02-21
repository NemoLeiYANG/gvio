#include <thread>
#include <signal.h>

#include "gvio/gvio.hpp"

using namespace gvio;

// GLOBAL VARIABLE
static volatile sig_atomic_t halt = 0;

void signal_handler(int signum) {
  UNUSED(signum);
  halt = 1;
}

struct recorder_state {
  bool pwm_ready = false;
  bool gimbal_ready = false;
  bool imu_ready = false;
  bool cam0_ready = false;
  bool cam1_ready = false;
  bool cam2_ready = false;
  bool record = false;
};

struct recorder_config {
  double pwm_frequency = 0.0;
  std::string gimbal_config;
  std::string imu_config;
  std::string cam0_config;
  std::string cam1_config;
  std::string cam2_config;
  std::string output_dir;
};

void print_usage() {
  std::cout << "Usage: recorder <recorder config dir>" << std::endl;
  std::cout << "Example: recorder recorder_config" << std::endl;
}

int recorder_load_config(const std::string &config_file,
                         struct recorder_config *config) {
  // Parse config file
  ConfigParser parser;
  parser.addParam("pwm_frequency", &config->pwm_frequency);
  parser.addParam("cam0_config", &config->cam0_config);
  parser.addParam("cam1_config", &config->cam1_config);
  parser.addParam("cam2_config", &config->cam2_config);
  parser.addParam("gimbal_config", &config->gimbal_config);
  parser.addParam("imu_config", &config->imu_config);
  parser.addParam("output_dir", &config->output_dir);
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
  }

  // Create output directory if not already created
  if (dir_create(config->output_dir) != 0) {
    LOG_ERROR("Failed to create output dir [%s]!", config->output_dir.c_str());
  }

  // Get dirname of config file path
  char str_buffer[100];
  memset(str_buffer, 0, 100); // Paranoid, just to be sure
  config_file.copy(str_buffer, config_file.length(), 0);
  const std::string config_path(dirname(str_buffer));

  // Correct paths
  paths_combine(config_path, config->cam0_config, config->cam0_config);
  paths_combine(config_path, config->cam1_config, config->cam1_config);
  paths_combine(config_path, config->cam2_config, config->cam2_config);
  paths_combine(config_path, config->gimbal_config, config->gimbal_config);
  paths_combine(config_path, config->imu_config, config->imu_config);

  return 0;
}

void recorder_is_ready(struct recorder_state *state) {
  if (state->pwm_ready == true && state->gimbal_ready == true &&
      state->imu_ready == true && state->cam0_ready == true &&
      state->cam1_ready == true && state->cam2_ready == true) {
    state->record = true;
  }
}

void pwm_thread(const double freq, struct recorder_state *state) {
  // Setup PWM signal
  PCA9685 pwm_driver;
  if (pwm_driver.configure(freq) != 0) {
    LOG_ERROR("Failed to configure PWM driver!");
    halt = 1;
    return;
  }
  pwm_driver.setAllPWM(4096 / 2);

  // Set PWM signal as ready
  state->pwm_ready = true;
}

void gimbal_thread(const std::string config_file,
                   struct recorder_state *state,
                   const std::string output_dir) {
  // Setup Gimbal
  Gimbal gimbal;
  if (gimbal.configure(config_file) != 0) {
    LOG_ERROR("Failed to configure gimbal!");
    halt = 1;
    return;
  }

  // Set Gimbal as ready
  state->gimbal_ready = true;

  // Setup output file
  const std::string output_path = output_dir + "/gimbal.dat";
  std::ofstream output_file(output_path);
  if (output_file.good() == false) {
    LOG_ERROR("Failed to create output file [%s]", output_path.c_str());
    halt = 1;
  }
  output_file << "t, roll, pitch, yaw" << std::endl;

  // Loop settings
  const double frequency = 30.0;
  double t_prev = 0.0;
  double t = 0.0;

  // Loop
  while (halt != 1) {
    // Calculate loop rate
    const double t_now = time_now();
    t += t_now - t_prev;
    t_prev = t_now;
    if (t < (1.0 / frequency)) {
      continue;
    }

    // Update gimbal
    gimbal.update();

    // Record gimbal measurements
    recorder_is_ready(state);
    if (state->record) {
      output_file << time_now() << ",";
      output_file << gimbal.camera_angles(0) << ",";
      output_file << gimbal.camera_angles(1) << ",";
      output_file << gimbal.camera_angles(2) << std::endl;
    }
  }
  output_file.close();
}

void imu_thread(const std::string config_file,
                struct recorder_state *state,
                const std::string output_dir) {
  // Setup IMU
  MPU6050 imu;
  if (imu.configure(config_file) != 0) {
    LOG_ERROR("Failed to configure imu!");
    halt = 1;
    return;
  }

  // Set IMU as ready
  state->imu_ready = true;

  // Setup output file
  const std::string output_path = output_dir + "/imu.dat";
  std::ofstream output_file(output_path);
  if (output_file.good() == false) {
    LOG_ERROR("Failed to create output file [%s]", output_path.c_str());
    halt = 1;
  }
  output_file << "t, ax_B, ay_B, az_B, wx_B, wy_B, wz_B" << std::endl;

  // Loop
  while (halt != 1) {
    // Get imu data
    imu.getData();

    // Record IMU measurements
    recorder_is_ready(state);
    if (state->record) {
      output_file << time_now() << ",";
      output_file << imu.accel.x << ",";
      output_file << imu.accel.y << ",";
      output_file << imu.accel.z << ",";
      output_file << imu.gyro.x << ",";
      output_file << imu.gyro.y << ",";
      output_file << imu.gyro.z << std::endl;
    }
  }
  output_file.close();
}

void camera_thread(const int camera_index,
                   const std::string config_file,
                   struct recorder_state *state,
                   const std::string output_dir) {
  // Setup camera
  IDSCamera camera;
  if (camera.configure(config_file) != 0) {
    LOG_ERROR("Failed to configure camera!");
    halt = 1;
    return;
  }

  // Set camera as ready
  switch (camera_index) {
    case 0: state->cam0_ready = true; break;
    case 1: state->cam1_ready = true; break;
    case 2: state->cam2_ready = true; break;
  }

  // Create output directory if not already created
  std::string output_path = output_dir;
  output_path += "/cam" + std::to_string(camera_index);
  if (dir_create(output_path) != 0) {
    LOG_ERROR("Failed to create output path [%s]!", output_path.c_str());
  }

  // Loop
  while (halt != 1) {
    // Get camera frame
    cv::Mat image;
    if (camera.getFrame(image) != 0) {
      break;
    }

    // Record
    recorder_is_ready(state);
    if (state->record) {
      const double t = time_now();
      std::string image_path = output_path + "/";
      image_path += std::to_string(t) + ".png";
      cv::imwrite(image_path, image);
    }
  }
}

int main(const int argc, const char *argv[]) {
  // Parse CLI args
  if (argc != 2) {
    print_usage();
    exit(-1);
  }
  std::string config_file(argv[1]);

  // Register signal handling
  signal(SIGINT, signal_handler);

  // Load recorder settings
  struct recorder_config config;
  if (recorder_load_config(config_file, &config) != 0) {
    LOG_ERROR("Failed to load recorder configs [%s]!", config_file.c_str());
    return -1;
  }

  // Launch threads
  // clang-format off
  struct recorder_state state;
  std::vector<std::thread> threads;
  threads.emplace_back(pwm_thread, config.pwm_frequency, &state);
  threads.emplace_back(gimbal_thread, config.gimbal_config, &state, config.output_dir);
  threads.emplace_back(imu_thread, config.imu_config, &state, config.output_dir);
  threads.emplace_back(camera_thread, 0, config.cam0_config, &state, config.output_dir);
  threads.emplace_back(camera_thread, 1, config.cam1_config, &state, config.output_dir);
  threads.emplace_back(camera_thread, 2, config.cam2_config, &state, config.output_dir);
  // clang-format on

  // Join threads
  for (auto &thread : threads) {
    thread.join();
  }

  return 0;
}
