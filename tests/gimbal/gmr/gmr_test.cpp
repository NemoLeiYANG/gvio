#include "gvio/munit.hpp"
#include "gvio/camera/camera.hpp"
#include "gvio/gimbal/gmr/gmr.hpp"

namespace gvio {

#define TEST_CONFIG_PATH "tests/test_configs/camera/webcam"

int test_GMR_GetSal() {
  GMR gmr;

  Camera camera;
  camera.configure(TEST_CONFIG_PATH);
  camera.connect();

  cv::Mat frame;
  cv::Mat saliency_map;

  while (true) {
    camera.getFrame(frame);
    saliency_map = gmr.GetSal(frame);

    cv::imshow("Image", frame);
    cv::imshow("Saliency", saliency_map);
    if (cv::waitKey(1) == 113) {
      break;
    }
  }

  return 0;
}

void test_suite() { MU_ADD_TEST(test_GMR_GetSal); }

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
