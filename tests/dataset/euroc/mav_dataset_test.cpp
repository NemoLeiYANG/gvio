#include "gvio/munit.hpp"
#include "gvio/dataset/euroc/mav_dataset.hpp"

namespace gvio {

#define TEST_DATA "test_data/euroc"

int mono_camera_cb(const cv::Mat &frame, const long ts) {
  UNUSED(ts);
  UNUSED(frame);
  // cv::imshow("Mono camera", frame);
  // cv::waitKey(1);
  return 0;
}

int stereo_camera_cb(const cv::Mat &frame0,
                     const cv::Mat &frame1,
                     const long ts) {
  cv::Mat stereo_img;
  cv::hconcat(frame0, frame1, stereo_img);

  UNUSED(ts);
  cv::imshow("Stereo", stereo_img);
  cv::waitKey(1);
  return 0;
}

int test_MAVDataset_constructor() {
  MAVDataset mav_data("/tmp");

  MU_CHECK(mav_data.ok == false);
  MU_CHECK_EQ("/tmp", mav_data.data_path);

  return 0;
}

int test_MAVDataset_loadIMUData() {
  MAVDataset mav_data(TEST_DATA);
  int retval = mav_data.loadIMUData();

  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_MAVDataset_loadCameraData() {
  MAVDataset mav_data(TEST_DATA);
  int retval = mav_data.loadCameraData();

  // Get timestamps and calculate relative time
  auto it = mav_data.timeline.begin();
  auto it_end = mav_data.timeline.end();
  while (it != it_end) {
    const long ts = it->first;
    mav_data.timestamps.push_back(ts);

    // Advance to next non-duplicate entry.
    do {
      ++it;
    } while (ts == it->first);
  }

  MU_CHECK_EQ(0, retval);
  MU_CHECK_EQ(10, mav_data.cam0_data.timestamps.size());
  MU_CHECK_EQ(10, mav_data.cam1_data.timestamps.size());
  MU_CHECK_EQ(10, mav_data.timestamps.size());

  return 0;
}

int test_MAVDataset_loadGroundTruthData() {
  MAVDataset mav_data(TEST_DATA);
  int retval = mav_data.loadGroundTruth();

  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_MAVDataset_load() {
  MAVDataset mav_data(TEST_DATA);

  int retval = mav_data.load();
  mav_data.mono_camera_cb = mono_camera_cb;
  // mav_data.stereo_camera_cb = stereo_camera_cb;
  mav_data.run();

  MU_CHECK_EQ(0, retval);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_MAVDataset_constructor);
  MU_ADD_TEST(test_MAVDataset_loadIMUData);
  MU_ADD_TEST(test_MAVDataset_loadCameraData);
  MU_ADD_TEST(test_MAVDataset_loadGroundTruthData);
  MU_ADD_TEST(test_MAVDataset_load);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
