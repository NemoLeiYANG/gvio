#include "gvio/gvio_test.hpp"
#include "gvio/kitti/kitti.hpp"

namespace gvio {

TEST(CalibCamToCam, load) {
  CalibCamToCam calib;

  calib.load("/data/raw/2011_09_26/calib_cam_to_cam.txt");

  const Vec2 S_00_exp{1.392000e+03, 5.120000e+02};
  // const Mat3 K_00_exp;
  // K_00_exp << 9.842439e+02, 0.000000e+00, 6.900000e+02,
  //             0.000000e+00, 9.808141e+02, 2.331966e+02,
  //             0.000000e+00, 0.000000e+00, 1.000000e+00;

  EXPECT_EQ("09-Jan-2012 13:57:47", calib.calib_time);
  EXPECT_FLOAT_EQ(9.950000e-02, calib.corner_dist);
}

TEST(RawDataset, construct) {
  RawDataset raw_dataset("/data/raw", "2011_09_26", "0005");
}

TEST(RawDataset, load) {
  RawDataset raw_dataset("/data/raw", "2011_09_26", "0005");

  raw_dataset.load();
}

} // namespace atl
