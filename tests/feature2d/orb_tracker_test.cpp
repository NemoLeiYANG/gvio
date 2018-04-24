#include "gvio/munit.hpp"
#include "gvio/dataset/kitti/kitti.hpp"
#include "gvio/feature2d/orb_tracker.hpp"

namespace gvio {

static const std::string KITTI_RAW_DATASET = "/data/kitti/raw";

int test_ORBTracker_update() {
  ORBTracker tracker;

  // Load dataset
  RawDataset raw_dataset(KITTI_RAW_DATASET, "2011_09_26", "0001", "extract");
  if (raw_dataset.load() != 0) {
    return -1;
  }

  // cv::VideoCapture capture(0);
  // cv::Mat img0;
  // capture >> img0;

  tracker.show_matches = true;
  tracker.initialize(cv::imread(raw_dataset.cam0[0]));
  // tracker.initialize(img0);

  for (int i = 1; i < 100; i++) {
    cv::Mat image = cv::imread(raw_dataset.cam0[i]);

    tracker.update(image);
    // std::cout << "buffer size: " << tracker.features.buffer.size() <<
    // std::endl;
    // std::cout << "lost size: " << tracker.features.lost.size() << std::endl;
    // std::cout << "tracking size: " << tracker.features.tracking.size()
    //           << std::endl;
    // std::cout << std::endl;

    // Break loop if 'q' was pressed
    if (tracker.show_matches && cv::waitKey(1) == 113) {
      break;
    }
  }

  return 0;
}

int test_ORBTracker_getLostTracks() {
  // Load raw dataset
  RawDataset raw_dataset(KITTI_RAW_DATASET, "2011_09_26", "0001");
  if (raw_dataset.load() != 0) {
    LOG_ERROR("Failed to load KITTI raw dataset [%s]!",
              KITTI_RAW_DATASET.c_str());
    return -1;
  }

  // Track features
  ORBTracker tracker;
  tracker.initialize(cv::imread(raw_dataset.cam0[0]));
  tracker.update(cv::imread(raw_dataset.cam0[1]));
  tracker.update(cv::imread(raw_dataset.cam0[2]));
  std::cout << tracker.features.lost.size() << std::endl;

  // Get lost tracks
  FeatureTracks tracks = tracker.getLostTracks();

  // Assert
  MU_CHECK(tracks.size() > 0);
  for (auto track : tracks) {
    MU_CHECK_EQ(tracker.features.buffer.find(track.track_id),
                tracker.features.buffer.end());
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_ORBTracker_update);
  // MU_ADD_TEST(test_ORBTracker_getLostTracks);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
