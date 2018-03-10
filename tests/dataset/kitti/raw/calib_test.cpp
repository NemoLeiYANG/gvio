#include "gvio/munit.hpp"
#include "gvio/dataset/kitti/kitti.hpp"

namespace gvio {

#define TEST_DATA "test_data/kitti/raw/2011_09_26/calib_cam_to_cam.txt"

int test_Calib_load() {
  CalibCamToCam calib;

  int retval = calib.load(TEST_DATA);
  MU_CHECK_EQ(retval, 0);

  const Vec2 S_00_exp{1.392000e+03, 5.120000e+02};
  MU_CHECK_EQ("09-Jan-2012 13:57:47", calib.calib_time);
  MU_CHECK_FLOAT(9.950000e-02, calib.corner_dist);

  return 0;
}

void test_suite() { MU_ADD_TEST(test_Calib_load); }

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
