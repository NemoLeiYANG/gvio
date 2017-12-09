#include "gvio/gvio_test.hpp"
#include "gvio/kitti/kitti.hpp"

namespace gvio {

TEST(CalibCamToCam, load) {
  CalibCamToCam calib;

  calib.load("/data/kitti/raw/2011_09_26/calib_cam_to_cam.txt");

  const Vec2 S_00_exp{1.392000e+03, 5.120000e+02};
  EXPECT_EQ("09-Jan-2012 13:57:47", calib.calib_time);
  EXPECT_FLOAT_EQ(9.950000e-02, calib.corner_dist);
}

} // namespace gvio
