#include <sys/time.h>

#include "gvio/gvio_test.hpp"
#include "gvio/feature2d/gms_matcher.hpp"

namespace gvio {

TEST(GMSMatcher, demo) {
  // Setup ORB
  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  orb->setFastThreshold(0);
  orb->setMaxFeatures(500);

  // Get camera frame
  cv::VideoCapture capture(1);
  cv::Mat img1;
  capture >> img1;

  // Detect and extract features
  std::vector<cv::KeyPoint> kp1;
  cv::Mat d1;
  orb->detectAndCompute(img1, cv::Mat(), kp1, d1);

  // Start timer
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int start = tp.tv_sec * 1000 + tp.tv_usec / 1000;

  while (cv::waitKey(1) != 113) {
    // Get camera frame
    cv::Mat img2;
    capture >> img2;

    // Detect and extract features
    std::vector<cv::KeyPoint> kp2;
    cv::Mat d2;
    orb->detectAndCompute(img2, cv::Mat(), kp2, d2);

    // BF match features
    std::vector<cv::DMatch> matches_all;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(d1, d2, matches_all);

    // GMS match
    std::vector<bool> vbInliers;
    GMSMatcher gms(kp1, img1.size(), kp2, img2.size(), matches_all);
    int nb_inliers = gms.GetInlierMask(vbInliers, false, false);

    std::vector<cv::DMatch> matches_gms;
    for (size_t i = 0; i < vbInliers.size(); i++) {
      if (vbInliers[i] == true) {
        matches_gms.push_back(matches_all[i]);
      }
    }

    cv::Mat matches_img = DrawInlier(img1, img2, kp1, kp2, matches_gms, 1);
    cv::imshow("Matches", matches_img);

    // Update old with new
    img2.copyTo(img1);
    kp1.clear();
    kp1 = kp2;
    d2.copyTo(d1);

    // Show FPS
    gettimeofday(&tp, NULL);
    long int stop = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    std::cout << "FPS: " << 1.0 / ((stop - start) / 1000.0) << "\t";
    std::cout << "Matches: " << nb_inliers << std::endl;
    start = stop;
  }
}

} // namespace gvio
