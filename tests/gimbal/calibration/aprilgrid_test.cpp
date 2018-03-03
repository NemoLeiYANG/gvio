#include "gvio/munit.hpp"
#include "gvio/gimbal/calibration/aprilgrid.hpp"

#define TEST_IMAGE "test_data/calibration/cam0/0.jpg"

namespace gvio {

int test_AprilGrid_constructor() {
  AprilGrid grid;

  std::cout << "black border: " << grid.detector.thisTagFamily.blackBorder
            << std::endl;

  auto capture = cv::VideoCapture(0);
  // const cv::Mat image = cv::imread(TEST_IMAGE);
  if (capture.isOpened() == false) {
    return -1;
  }

  while (true) {
    cv::Mat image;
    capture.read(image);

    grid.extractCorners(image);
    cv::imshow("Image", image);
    if (cv::waitKey(1) == 113) {
      break;
    }
  }

  // cv::imshow("Image", image);
  // cv::waitKey(1);

  return 0;
}

void test_suite() { MU_ADD_TEST(test_AprilGrid_constructor); }

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
