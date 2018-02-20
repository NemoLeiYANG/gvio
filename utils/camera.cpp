#include "gvio/gvio.hpp"

using namespace gvio;

void print_usage() {
  std::cout << "Usage: camera <camera config>" << std::endl;
  std::cout << "Example: camera camera.yaml" << std::endl;
}

int main(const int argc, const char *argv[]) {
  // Parse CLI args
  if (argc != 2) {
    print_usage();
    exit(-1);
  }
  std::string camera_config(argv[1]);

  // Setup camera
  IDSCamera camera;
  camera.configure(camera_config);

  // Listen for camera images
  const std::string win_name = "Camera " + std::to_string(camera.camera_handle);
  while (true) {
    cv::Mat image;
    if (camera.getFrame(image) != 0) {
      return -1;
    }

    cv::imshow(win_name, image);
    if (cv::waitKey(1) == 113) { // Break loop if 'q' is pressed
      break;
    }
  }

  return 0;
}
