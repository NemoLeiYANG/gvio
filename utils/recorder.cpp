#include <iostream>
#include <thread>
#include <string>

#include <libgen.h>

#include "gvio/gvio.hpp"
#include "gvio/imu/mpu6050.hpp"

using namespace gvio;

struct recorder_state {
  bool pwm_ready = false;
  bool gimbal_ready = false;
  bool imu_ready = false;
  bool cam0_ready = false;
  bool cam1_ready = false;
  bool cam2_ready = false;
  bool record = false;
  bool halt = false;
};

struct recorder_config {
  double pwm_frequency = 0;
  std::string gimbal_config;
  std::string imu_config;
  std::string cam0_config;
  std::string cam1_config;
  std::string cam2_config;
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
  if (parser.load(config_file) != 0) {
    LOG_ERROR("Failed to load config file [%s]!", config_file.c_str());
    return -1;
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

void pwm_thread(const double freq, struct recorder_state *state) {
  // Setup PWM signal
  PCA9685 pwm_driver;
  if (pwm_driver.configure(freq) != 0) {
    LOG_ERROR("Failed to configure PWM driver!");
    state->halt = true;
    return;
  }
  pwm_driver.setAllPWM(4096 / 2);

  // Set PWM signal as ready
  state->pwm_ready = true;
}

void gimbal_thread(const std::string config_file,
                   struct recorder_state *state) {
  // Setup Gimbal
  Gimbal gimbal;
  if (gimbal.configure(config_file) != 0) {
    LOG_ERROR("Failed to configure gimbal!");
    state->halt = true;
    return;
  }

  // Set Gimbal as ready
  state->gimbal_ready = true;

  // Loop settings
  const double frequency = 30.0;
  double t_prev = 0.0;
  double t = 0.0;

  // Loop
  while (state->halt == false) {
    // Calculate loop rate
    const double t_now = time_now();
    t += t_now - t_prev;
    t_prev = t_now;
    if (t < (1.0 / frequency)) {
      continue;
    }

    // Update gimbal
    gimbal.update();
  }
}

void imu_thread(const std::string config_file, struct recorder_state *state) {
  // Setup IMU
  MPU6050 imu;
  if (imu.configure(config_file) != 0) {
    LOG_ERROR("Failed to configure imu!");
    state->halt = true;
    return;
  }

  // Set IMU as ready
  state->imu_ready = true;

  // Loop
  while (state->halt == false) {
    imu.getData();

    std::cout << imu.gyro.x << "\t";
    std::cout << imu.gyro.y << "\t";
    std::cout << imu.gyro.z << std::endl;

    std::cout << imu.accel.x << "\t";
    std::cout << imu.accel.y << "\t";
    std::cout << imu.accel.z << std::endl;

    std::cout << std::endl;
  }
}

void camera_thread(const int camera_index,
                   const std::string config_file,
                   struct recorder_state *state) {
  // Setup camera
  IDSCamera camera;
  if (camera.configure(config_file) != 0) {
    LOG_ERROR("Failed to configure camera!");
    state->halt = true;
    return;
  }

  // Set camera as ready
  switch (camera_index) {
    case 0: state->cam0_ready = true; break;
    case 1: state->cam1_ready = true; break;
    case 2: state->cam2_ready = true; break;
  }

  // Loop
  while (state->halt == false) {
    cv::Mat image;
    if (camera.getFrame(image) != 0) {
      break;
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

  // Load recorder settings
  struct recorder_config config;
  if (recorder_load_config(config_file, &config) != 0) {
    LOG_ERROR("Failed to load recorder configs [%s]!", config_file.c_str());
    return -1;
  }

  // Launch threads
  std::vector<std::thread> threads;
  struct recorder_state state;

  threads.emplace_back(pwm_thread, config.pwm_frequency, &state);
  threads.emplace_back(gimbal_thread, config.gimbal_config, &state);
  threads.emplace_back(imu_thread, config.imu_config, &state);
  threads.emplace_back(camera_thread, 0, config.cam0_config, &state);
  threads.emplace_back(camera_thread, 1, config.cam1_config, &state);
  threads.emplace_back(camera_thread, 2, config.cam2_config, &state);

  // Join threads
  for (auto &thread : threads) {
    thread.join();
  }

  return 0;
}
