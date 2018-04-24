#include "gvio/munit.hpp"
#include "gvio/feature2d/grid_fast.hpp"

namespace gvio {

#define TEST_IMAGE_CENTER "test_data/apriltag/center.png"

int test_grid_fast() {
  const cv::Mat image = cv::imread(TEST_IMAGE_CENTER);
  bool debug = true;

  grid_fast(image,  // Input image
            1000,   // Max number of corners
            5,      // Grid rows
            5,      // Grid columns
            10.0,   // Threshold
            true,   // Nonmax suppression
            debug); // Debug
  if (debug) {
    cv::waitKey(0);
  }

  return 0;
}

void test_suite() { MU_ADD_TEST(test_grid_fast); }

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
