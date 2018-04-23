#include "gvio/munit.hpp"
#include "gvio/dataset/kitti/kitti.hpp"
#include "gvio/feature2d/stereo_orb_tracker.hpp"

namespace gvio {

// static const std::string KITTI_RAW_DATASET = "test_data/kitti/raw";
static const std::string KITTI_RAW_DATASET = "/data/kitti/raw";

int test_StereoORBTracker_update() {
  CameraProperty camprop0;
  CameraProperty camprop1;
  StereoORBTracker tracker(camprop0, camprop1, 1, 5);

  // Load dataset
  RawDataset raw_dataset(KITTI_RAW_DATASET, "2011_09_26", "0001", "extract");
  if (raw_dataset.load() != 0) {
    return -1;
  }

  // Initialize tracker
  const cv::Mat img0 = cv::imread(raw_dataset.cam0[0]);
  const cv::Mat img1 = cv::imread(raw_dataset.cam1[0]);
  tracker.show_matches = false;

  // Update tracker
  for (int i = 0; i < 5; i++) {
    const cv::Mat img0 = cv::imread(raw_dataset.cam0[i]);
    const cv::Mat img1 = cv::imread(raw_dataset.cam1[i]);
    tracker.update(img0, img1);

    // Check tracks in tracker0 are related to tracks in tracker1
    for (auto track_id : tracker.tracker0.features.tracking) {
      auto id = tracker.tracker0.features.buffer[track_id].related;
      auto buffer = tracker.tracker1.features.buffer;
      auto track = tracker.tracker0.features.buffer[track_id];

      MU_CHECK_EQ(1, buffer.count(id));
      MU_CHECK_EQ(track_id, tracker.tracker1.features.buffer[id].related);
    }

    // Check tracks in tracker1 are related to tracks in tracker0
    for (auto track_id : tracker.tracker1.features.tracking) {
      auto id = tracker.tracker1.features.buffer[track_id].related;
      auto buffer = tracker.tracker0.features.buffer;
      auto track = tracker.tracker1.features.buffer[track_id];

      MU_CHECK_EQ(1, buffer.count(id));
      MU_CHECK_EQ(track_id, tracker.tracker0.features.buffer[id].related);
    }

    // Break loop if 'q' was pressed
    if (tracker.show_matches && cv::waitKey(0) == 113) {
      break;
    }
  }

  return 0;
}

int test_StereoORBTracker_getLostTracks() {
  // Load raw dataset
  RawDataset raw_dataset(KITTI_RAW_DATASET, "2011_09_26", "0001", "extract");
  if (raw_dataset.load() != 0) {
    LOG_ERROR("Failed to load KITTI raw dataset [%s]!",
              KITTI_RAW_DATASET.c_str());
    return -1;
  }

  // Track features
  CameraProperty camprop0;
  CameraProperty camprop1;
  StereoORBTracker tracker(camprop0, camprop1, 1, 5);
  tracker.show_matches = false;

  const cv::Mat img0 = cv::imread(raw_dataset.cam0[0]);
  const cv::Mat img1 = cv::imread(raw_dataset.cam1[0]);

  for (int i = 0; i < 5; i++) {
    const cv::Mat img0 = cv::imread(raw_dataset.cam0[i]);
    const cv::Mat img1 = cv::imread(raw_dataset.cam1[i]);
    tracker.update(img0, img1);

    // Get lost tracks
    FeatureTracks stereo_tracks = tracker.getLostTracks();

    std::cout << "lost tracks: " << stereo_tracks.size() << std::endl;
    std::cout << "tracker0 tracking: "
              << tracker.tracker0.features.tracking.size() << std::endl;
    std::cout << "tracker1 tracking: "
              << tracker.tracker1.features.tracking.size() << std::endl;
    std::cout << "tracker0 lost: " << tracker.tracker0.features.lost.size()
              << std::endl;
    std::cout << "tracker1 lost: " << tracker.tracker1.features.lost.size()
              << std::endl;
    std::cout << "tracks lost: " << stereo_tracks.size() << std::endl;
    std::cout << "tracker0 buffer: " << tracker.tracker0.features.buffer.size()
              << std::endl;
    std::cout << "tracker1 buffer: " << tracker.tracker1.features.buffer.size()
              << std::endl;
    std::cout << std::endl;
  }

  // Assert
  // MU_CHECK(stereo_tracks.size() > 0);
  // for (auto track : stereo_tracks) {
  //   MU_CHECK_EQ(track.track0.size(), track.track1.size());
  // }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_StereoORBTracker_update);
  // MU_ADD_TEST(test_StereoORBTracker_getLostTracks);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
