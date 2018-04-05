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

  // Setup IMU
  MPU6050 imu;
  imu.configure(imu_config);

  while (true) {
    imu.getData();

    std::cout << imu.gyro.x << "\t";
    std::cout << imu.gyro.y << "\t";
    std::cout << imu.gyro.z << std::endl;

    std::cout << imu.accel.x << "\t";
    std::cout << imu.accel.y << "\t";
    std::cout << imu.accel.z << std::endl;

    std::cout << std::endl;
  }

  return 0;
}
