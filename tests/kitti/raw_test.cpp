#include "gvio/gvio_test.hpp"
#include "gvio/kitti/kitti.hpp"

namespace gvio {

TEST(KITTI, parseString) {
  std::string value = parseString("X: Hello World");

  EXPECT_EQ("Hello World", value);
}

TEST(KITTI, parseDouble) {
  double value = parseDouble("X: 1.23");

  EXPECT_FLOAT_EQ(1.23, value);
}

TEST(KITTI, parseArray) {
  std::vector<double> value = parseArray("X: 1.0 2.0 3.0");

  EXPECT_EQ(3, (int) value.size());
  EXPECT_FLOAT_EQ(1.0, value[0]);
  EXPECT_FLOAT_EQ(2.0, value[1]);
  EXPECT_FLOAT_EQ(3.0, value[2]);
}

TEST(KITTI, parseVec2) {
  Vec2 value = parseVec2("X: 1.0 2.0");

  EXPECT_FLOAT_EQ(1.0, value(0));
  EXPECT_FLOAT_EQ(2.0, value(1));
}

TEST(KITTI, parseVec3) {
  Vec3 value = parseVec3("X: 1.0 2.0 3.0");

  EXPECT_FLOAT_EQ(1.0, value(0));
  EXPECT_FLOAT_EQ(2.0, value(1));
  EXPECT_FLOAT_EQ(3.0, value(2));
}

TEST(KITTI, parseVecX) {
  VecX value = parseVecX("X: 1.0 2.0 3.0 4.0 5.0 6.0");

  EXPECT_FLOAT_EQ(1.0, value(0));
  EXPECT_FLOAT_EQ(2.0, value(1));
  EXPECT_FLOAT_EQ(3.0, value(2));
  EXPECT_FLOAT_EQ(4.0, value(3));
  EXPECT_FLOAT_EQ(5.0, value(4));
  EXPECT_FLOAT_EQ(6.0, value(5));
}

TEST(KITTI, parseMat3) {
  Mat3 value = parseMat3("X: 1 2 3 4 5 6 7 8 9");

  EXPECT_FLOAT_EQ(1.0, value(0, 0));
  EXPECT_FLOAT_EQ(2.0, value(0, 1));
  EXPECT_FLOAT_EQ(3.0, value(0, 2));
  EXPECT_FLOAT_EQ(4.0, value(1, 0));
  EXPECT_FLOAT_EQ(5.0, value(1, 1));
  EXPECT_FLOAT_EQ(6.0, value(1, 2));
  EXPECT_FLOAT_EQ(7.0, value(2, 0));
  EXPECT_FLOAT_EQ(8.0, value(2, 1));
  EXPECT_FLOAT_EQ(9.0, value(2, 2));
}

TEST(KITTI, parseMat34) {
  Mat34 value = parseMat34("X: 1 2 3 4 5 6 7 8 9 10 11 12");

  EXPECT_FLOAT_EQ(1.0, value(0, 0));
  EXPECT_FLOAT_EQ(2.0, value(0, 1));
  EXPECT_FLOAT_EQ(3.0, value(0, 2));
  EXPECT_FLOAT_EQ(4.0, value(0, 3));

  EXPECT_FLOAT_EQ(5.0, value(1, 0));
  EXPECT_FLOAT_EQ(6.0, value(1, 1));
  EXPECT_FLOAT_EQ(7.0, value(1, 2));
  EXPECT_FLOAT_EQ(8.0, value(1, 3));

  EXPECT_FLOAT_EQ(9.0, value(2, 0));
  EXPECT_FLOAT_EQ(10.0, value(2, 1));
  EXPECT_FLOAT_EQ(11.0, value(2, 2));
  EXPECT_FLOAT_EQ(12.0, value(2, 3));
}

TEST(CalibCamToCam, load) {
  CalibCamToCam calib;

  calib.load("/data/raw/2011_09_26/calib_cam_to_cam.txt");

  const Vec2 S_00_exp{1.392000e+03, 5.120000e+02};

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
