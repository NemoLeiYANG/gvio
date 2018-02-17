#include "gvio/gvio.hpp"

using namespace gvio;

void print_usage() {
  std::cout << "Usage: imu <imu config>" << std::endl;
  std::cout << "Example: imu imu.yaml" << std::endl;
}

int main(const int argc, const char *argv[]) {
  // Parse CLI args
  if (argc != 2) {
    print_usage();
    exit(-1);
  }
  std::string imu_config(argv[1]);

  // Setup camera
  IDSCamera camera;
  camera.configure(imu_config);

  return 0;
}
