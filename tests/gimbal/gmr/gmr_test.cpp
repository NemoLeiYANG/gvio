#include "gvio/munit.hpp"
#include "gvio/camera/camera.hpp"
#include "gvio/gimbal/gmr/gmr.hpp"

#include "opencv2/features2d/features2d.hpp"

namespace gvio {

#define TEST_CONFIG_PATH "tests/test_configs/camera/webcam"

std::string cvmattype2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch (depth) {
    case CV_8U: r = "8U"; break;
    case CV_8S: r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default: r = "User"; break;
  }

  r += "C";
  r += (chans + '0');

  return r;
}

int test_GMR_GetSal() {
  GMR gmr;

  Camera camera;
  camera.configure(TEST_CONFIG_PATH);
  camera.connect();

  cv::Mat frame;
  while (true) {
    camera.getFrame(frame);
    const cv::Mat saliency_map = gmr.GetSal(frame);

    // Threshold saliency map
    cv::Mat saliency_threshold;
    cv::threshold(saliency_map,
                  saliency_threshold,
                  0.95,
                  1.0,
                  cv::THRESH_BINARY);

    // Convert from 32FC1 to 8UC1
    double val_min, val_max;
    cv::minMaxLoc(saliency_threshold,
                  &val_min,
                  &val_max); // find  minimum  and  maximum  intensities
    saliency_threshold.convertTo(saliency_threshold,
                                 CV_8U,
                                 255.0 / (val_max - val_min),
                                 -val_min);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(saliency_threshold,
                     contours,
                     hierarchy,
                     cv::RETR_TREE,
                     cv::CHAIN_APPROX_SIMPLE,
                     cv::Point(0, 0));

    // Draw contours
    for (size_t i = 0; i < contours.size(); i++) {
      cv::Scalar color = cv::Scalar(255, 0, 0);
      drawContours(frame,
                   contours,
                   (int) i,
                   color,
                   2,
                   8,
                   hierarchy,
                   0,
                   cv::Point());
    }

    // Show images
    cv::imshow("Image", frame);
    cv::imshow("Saliency", saliency_map);
    cv::imshow("Saliency Threshold", saliency_threshold);
    if (cv::waitKey(1) == 113) {
      break;
    }
  }

  return 0;
}

void test_suite() { MU_ADD_TEST(test_GMR_GetSal); }

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
