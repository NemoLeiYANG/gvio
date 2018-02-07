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
  mav_data.loadIMUData();

  return 0;
}

int test_MAVDataset_load() {
  MAVDataset mav_data("/tmp");

  // mav_data.

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_MAVDataset_constructor);
  MU_ADD_TEST(test_MAVDataset_loadIMUData);
  MU_ADD_TEST(test_MAVDataset_load);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
