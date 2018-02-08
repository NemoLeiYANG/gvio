#include "gvio/munit.hpp"
#include "gvio/euroc/mav_dataset.hpp"

namespace gvio {

int test_MAVDataset_constructor() {
  MAVDataset mav_data("/tmp");

  MU_CHECK(mav_data.ok == false);
  MU_CHECK_EQ("/tmp", mav_data.data_path);

  return 0;
}

int test_MAVDataset_loadIMUData() {
  MAVDataset mav_data("/data/euroc_mav/raw/mav0");
  int retval = mav_data.loadIMUData();

  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_MAVDataset_loadCameraData() {
  MAVDataset mav_data("/data/euroc_mav/raw/mav0");
  int retval = mav_data.loadCameraData();

  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_MAVDataset_loadGroundTruthData() {
  MAVDataset mav_data("/data/euroc_mav/raw/mav0");
  int retval = mav_data.loadGroundTruthData();

  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_MAVDataset_load() {
  MAVDataset mav_data("/data/euroc_mav/raw/mav0");
  int retval = mav_data.load();

  std::cout << mav_data.cam0_data.timestamps.size() << std::endl;
  std::cout << mav_data.imu_data.timestamps.size() << std::endl;
  std::cout << mav_data.ground_truth.timestamps.size() << std::endl;
  std::cout << mav_data.ts_start << std::endl;
  std::cout << mav_data.ts_end << std::endl;

  for (long t = mav_data.ts_start; t < mav_data.ts_end; t++) {
    // printf("%ld\n", t);
  }

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
