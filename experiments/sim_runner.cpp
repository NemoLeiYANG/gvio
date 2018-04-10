#include "gvio/gvio.hpp"

using namespace gvio;

void print_usage() {
  // Usage
  std::cout << "Usage: sim_runner ";
  std::cout << "<sim_config_path> ";
  std::cout << "<msckf_config_path> " << std::endl;

  // Example
  std::cout << "Example: sim_runner ";
  std::cout << "sim.yaml ";
  std::cout << "msckf.yaml" << std::endl;
}

int main(const int argc, const char *argv[]) {
  // Parse cli args
  if (argc != 3) {
    print_usage();
    return -1;
  }
  const std::string sim_config_path(argv[1]);
  const std::string msckf_config_path(argv[2]);

  // Setup MSCKF
  MSCKF msckf;
  if (msckf.configure(msckf_config_path) != 0) {
    LOG_ERROR("Failed to configure MSCKF!");
    return -1;
  }

  // Setup simulation
  SimWorld sim;
  if (sim.configure(sim_config_path) != 0) {
    LOG_ERROR("Failed to configure simulation!");
    return -1;
  }

  // Initialize MSCKF
  msckf.initialize(sim.t,
                   euler2quat(sim.camera_motion.rpy_G),
                   sim.camera_motion.v_G,
                   sim.camera_motion.p_G);
  sim.recordEstimate(sim.t,
                     msckf.imu_state.p_G,
                     msckf.imu_state.v_G,
                     quat2euler(msckf.imu_state.q_IG));

  // Simulate
  while (sim.t <= sim.t_end) {
    sim.step();
    std::cout << "time index: " << sim.time_index << std::endl;

    const Vec3 a_m = sim.camera_motion.a_B;
    const Vec3 w_m = sim.camera_motion.w_B;
    msckf.predictionUpdate(a_m, w_m, sim.t * 1.0e9);

    const std::vector<FeatureTrack> tracks = sim.getLostTracks();
    msckf.measurementUpdate(tracks);

    sim.recordEstimate(sim.t,
                       msckf.imu_state.p_G,
                       msckf.imu_state.v_G,
                       quat2euler(msckf.imu_state.q_IG));
  }

  return 0;
}
