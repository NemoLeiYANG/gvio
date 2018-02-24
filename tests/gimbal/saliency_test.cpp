#include "gvio/munit.hpp"
#include "gvio/camera/camera.hpp"
#include "gvio/gimbal/saliency.hpp"

namespace gvio {

#define TEST_CONFIG_PATH "tests/test_configs/camera/webcam"

int test_Saliency_constructor() {
  Saliency saliency;
  return 0;
}

int test_Saliency_detect() {
  Saliency saliency;

  Camera camera;
  camera.configure(TEST_CONFIG_PATH);
  camera.connect();

  while (true) {
    cv::Mat frame;
    camera.getFrame(frame);

    saliency.detect(frame);

    cv::imshow("Frame", frame);
    if (cv::waitKey(1) == 113) {
      break;
    }
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_Saliency_constructor);
  MU_ADD_TEST(test_Saliency_detect);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
