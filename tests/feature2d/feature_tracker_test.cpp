#include "gvio/munit.hpp"
#include "gvio/feature2d/feature_tracker.hpp"

namespace gvio {

static const std::string TEST_IMAGE_TOP = "test_data/apriltag/top.png";
static const std::string TEST_IMAGE_BOTTOM = "test_data/apriltag/bottom.png";

class FeatureTrackerTester : public FeatureTracker {
public:
  cv::Ptr<cv::ORB> orb = cv::ORB::create();

  FeatureTrackerTester() {}
  virtual ~FeatureTrackerTester() {}

  virtual int detect(const cv::Mat &image, Features &features) override {
    // Feature descriptor extraction
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat mask;
    cv::Mat descriptors;
    this->orb->detectAndCompute(image, mask, keypoints, descriptors);

    // Update counters
    this->counter_frame_id += 1;

    // Create features
    for (int i = 0; i < descriptors.rows; i++) {
      features.emplace_back(keypoints[i], descriptors.row(i));
    }

    return 0;
  }
};

int test_FeatureTracker_constructor() {
  FeatureTrackerTester tracker;

  MU_CHECK(tracker.show_matches == false);
  MU_CHECK_EQ(-1, (int) tracker.counter_frame_id);

  return 0;
}

int test_FeatureTracker_conversions() {
  // Detect keypoints and descriptors
  FeatureTrackerTester tracker;
  const cv::Mat img = cv::imread(TEST_IMAGE_TOP, CV_LOAD_IMAGE_COLOR);
  std::vector<Feature> features;
  tracker.detect(img, features);

  // Convert features to keypoints and descriptors
  std::vector<cv::KeyPoint> k_converted;
  cv::Mat d_converted;
  tracker.getKeyPointsAndDescriptors(features, k_converted, d_converted);

  // Convert keypoints and descriptors to features
  std::vector<Feature> f_converted;
  tracker.getFeatures(k_converted, d_converted, f_converted);

  // Assert
  int index = 0;
  for (auto &f : f_converted) {
    MU_CHECK(f.kp.pt.x == k_converted[index].pt.x);
    MU_CHECK(f.kp.pt.y == k_converted[index].pt.y);

    MU_CHECK(f.kp.pt.x == features[index].kp.pt.x);
    MU_CHECK(f.kp.pt.y == features[index].kp.pt.y);

    MU_CHECK(is_equal(f.desc, d_converted.row(index)));

    MU_CHECK_EQ(f.desc.size(), cv::Size(32, 1));
    index++;
  }

  return 0;
}

int test_FeatureTracker_match() {
  FeatureTrackerTester tracker;
  const cv::Mat img0 = cv::imread(TEST_IMAGE_TOP, CV_LOAD_IMAGE_COLOR);
  const cv::Mat img1 = cv::imread(TEST_IMAGE_BOTTOM, CV_LOAD_IMAGE_COLOR);
  // cv::imshow("Image0", img0);
  // cv::imshow("Image1", img1);
  // cv::waitKey();

  // Detect features in image 0
  std::vector<Feature> f0;
  tracker.detect(img0, f0);
  tracker.fea_ref = f0;
  tracker.img_size = img0.size();

  // Detect features in image 1
  std::vector<Feature> f1;
  tracker.detect(img1, f1);

  // Perform matching
  tracker.show_matches = true;
  tracker.img_ref = img0;
  tracker.img_cur = img1;

  std::vector<cv::DMatch> matches;
  tracker.match(f1, matches);
  cv::waitKey();

  // Asserts
  MU_CHECK(matches.size() > 0);

  return 0;
}

int test_FeatureTracker_initialize() {
  FeatureTrackerTester tracker;
  return 0;
}

int test_FeatureTracker_update() {
  FeatureTrackerTester tracker;
  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_FeatureTracker_constructor);
  MU_ADD_TEST(test_FeatureTracker_conversions);
  MU_ADD_TEST(test_FeatureTracker_match);
  MU_ADD_TEST(test_FeatureTracker_initialize);
  MU_ADD_TEST(test_FeatureTracker_update);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
