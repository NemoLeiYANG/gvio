#include "gvio/munit.hpp"
#include "gvio/feature2d/grid_fast.hpp"

namespace gvio {

#define TEST_IMAGE_CENTER "test_data/apriltag/center.png"
#define TEST_IMAGE "test_data/euroc/cam0/data/1403715273262142976.png"

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

int benchmark_grid_fast() {
  // Grid-FAST corner detector
  {
    const cv::Mat image = cv::imread(TEST_IMAGE);
    auto keypoints = grid_fast(image, // Input image
                               1000,  // Max number of corners
                               5,     // Grid rows
                               5,     // Grid columns
                               10.0,  // Threshold
                               true,  // Nonmax suppression
                               true); // Debug

    // Save keypoints to file
    MatX data;
    data.resize(keypoints.size(), 2);
    int row_index = 0;
    for (auto kp : keypoints) {
      data(row_index, 0) = kp.pt.x;
      data(row_index, 1) = kp.pt.y;
      row_index++;
    }
    mat2csv("/tmp/grid_fast.csv", data);
    cv::imwrite("/tmp/grid_fast.png", image);
  }

  // Standard FAST corner detector
  {
    // Prepare input image - make sure it is grayscale
    const cv::Mat image = cv::imread(TEST_IMAGE);
    cv::Mat image_gray;
    if (image.channels() == 3) {
      cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    } else {
      image_gray = image.clone();
    }

    std::vector<cv::KeyPoint> keypoints;
    cv::FAST(image_gray, // Input image
             keypoints,  // Keypoints
             10.0,       // Threshold
             true);      // Nonmax suppression

    // Sort by keypoint response
    keypoints = sort_keypoints(keypoints, 1000);

    // Draw corners
    for (auto kp : keypoints) {
      cv::circle(image, kp.pt, 2, cv::Scalar(0, 255, 0), -1);
    }

    // Draw
    cv::imshow("FAST", image);
    cv::waitKey(1);

    // Save image and keypoints to file
    MatX data;
    data.resize(keypoints.size(), 2);
    int row_index = 0;
    for (auto kp : keypoints) {
      data(row_index, 0) = kp.pt.x;
      data(row_index, 1) = kp.pt.y;
      row_index++;
    }
    mat2csv("/tmp/fast.csv", data);
    cv::imwrite("/tmp/fast.png", image);
  }

  // Visualize results
  do {
  } while (cv::waitKey(0) != 113);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_grid_fast);
  MU_ADD_TEST(benchmark_grid_fast);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
