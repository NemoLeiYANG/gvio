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
  const std::string win_name = "Image" + std::to_string(rand() % 10);
  while (true) {
    cv::Mat image;
    if (camera.getFrame(image) != 0) {
      return -1;
    }

    cv::imshow(win_name, image);
    if (cv::waitKey(1) == 113) {
      break;
    }
  }

  return 0;
}
