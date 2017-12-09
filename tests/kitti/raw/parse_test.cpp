#include "gvio/gvio_test.hpp"
#include "gvio/kitti/raw/parse.hpp"

namespace gvio {

TEST(Parse, parseString) {
  std::string value = parseString("X: Hello World");

  EXPECT_EQ("Hello World", value);
}

TEST(Parse, parseDouble) {
  double value = parseDouble("X: 1.23");

  EXPECT_FLOAT_EQ(1.23, value);
}

TEST(Parse, parseArray) {
  std::vector<double> value = parseArray("X: 1.0 2.0 3.0");

  EXPECT_EQ(3, (int) value.size());
  EXPECT_FLOAT_EQ(1.0, value[0]);
  EXPECT_FLOAT_EQ(2.0, value[1]);
  EXPECT_FLOAT_EQ(3.0, value[2]);
}

TEST(Parse, parseVec2) {
  Vec2 value = parseVec2("X: 1.0 2.0");

  EXPECT_FLOAT_EQ(1.0, value(0));
  EXPECT_FLOAT_EQ(2.0, value(1));
}

TEST(Parse, parseVec3) {
  Vec3 value = parseVec3("X: 1.0 2.0 3.0");

  EXPECT_FLOAT_EQ(1.0, value(0));
  EXPECT_FLOAT_EQ(2.0, value(1));
  EXPECT_FLOAT_EQ(3.0, value(2));
}

TEST(Parse, parseVecX) {
  VecX value = parseVecX("X: 1.0 2.0 3.0 4.0 5.0 6.0");

  EXPECT_FLOAT_EQ(1.0, value(0));
  EXPECT_FLOAT_EQ(2.0, value(1));
  EXPECT_FLOAT_EQ(3.0, value(2));
  EXPECT_FLOAT_EQ(4.0, value(3));
  EXPECT_FLOAT_EQ(5.0, value(4));
  EXPECT_FLOAT_EQ(6.0, value(5));
}

TEST(Parse, parseMat3) {
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

TEST(Parse, parseMat34) {
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

} // namespace gvio
