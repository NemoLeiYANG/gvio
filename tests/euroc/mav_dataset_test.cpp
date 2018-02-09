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

  // // Get timestamps
  // std::vector<long> timestamps;
  // auto it = mav_data.timeline.begin();
  // auto it_end = mav_data.timeline.end();
  // while (it != it_end) {
  //   timestamps.push_back(it->first);
  //   it = mav_data.timeline.upper_bound(it->first);
  // }
  //
  // // Iterate through timestamps
  // long time_index = 0;
  // for (auto ts : timestamps) {
  //   std::cout << "time index: " << time_index << std::endl;
  //
  //   auto it = mav_data.timeline.lower_bound(ts);
  //   auto it_end = mav_data.timeline.upper_bound(ts);
  //   while (it != it_end) {
  //     std::cout << it->second << std::endl;
  //     it++;
  //   }
  //   std::cout << std::endl;
  //   time_index++;
  // }

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
