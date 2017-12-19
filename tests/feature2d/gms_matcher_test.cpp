#include <sys/time.h>

#include "gvio/munit.h"
#include "gvio/feature2d/gms_matcher.hpp"

namespace gvio {

int test_GMSMatcher_demo() {
  // Setup ORB
  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  orb->setFastThreshold(0);
  orb->setMaxFeatures(100);

  // Get camera frame
  cv::VideoCapture capture(0);
  cv::Mat img1;
  capture >> img1;

  // Detect and extract features
  std::vector<cv::KeyPoint> k1;
  cv::Mat d1;
  orb->detectAndCompute(img1, cv::Mat(), k1, d1);

  // Start timer
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int start = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  int frame_counter = 0;

  GMSMatcher gms;
  while (cv::waitKey(1) != 113) {
    // Get camera frame
    cv::Mat img2;
    capture >> img2;

    // Detect and extract features
    std::vector<cv::KeyPoint> k2;
    cv::Mat d2;
    orb->detectAndCompute(img2, cv::Mat(), k2, d2);

    // GMS match
    std::vector<cv::DMatch> matches_gms;
    const int nb_inliers = gms.match(k1, d1, k2, d2, img1.size(), matches_gms);
    cv::Mat matches_img = draw_inliers(img1, img2, k1, k2, matches_gms, 1);
    cv::imshow("Matches", matches_img);

    // Update old with new
    img2.copyTo(img1);
    k1.clear();
    k1 = k2;
    d2.copyTo(d1);

    // Show FPS
    frame_counter++;
    if (frame_counter % 10 == 0) {
      gettimeofday(&tp, NULL);
      long int stop = tp.tv_sec * 1000 + tp.tv_usec / 1000;
      std::cout << "FPS: " << 10.0 / ((stop - start) / 1000.0) << "\t";
      std::cout << "Matches: " << nb_inliers << std::endl;
      start = stop;

      frame_counter = 0;
    }
  }

  return 0;
}

void test_suite() {
  // MU_ADD_TEST(test_GMSMatcher_demo);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
