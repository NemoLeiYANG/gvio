#include "gvio/munit.hpp"
#include "gvio/kitti/raw/parse.hpp"

namespace gvio {

int test_Parse_parseString() {
  std::string value = parseString("X: Hello World");

  MU_CHECK_EQ("Hello World", value);

  return 0;
}

int test_Parse_parseDouble() {
  double value = parseDouble("X: 1.23");

  MU_CHECK_FLOAT(1.23, value);

  return 0;
}

int test_Parse_parseArray() {
  std::vector<double> value = parseArray("X: 1.0 2.0 3.0");

  MU_CHECK_EQ(3, (int) value.size());
  MU_CHECK_FLOAT(1.0, value[0]);
  MU_CHECK_FLOAT(2.0, value[1]);
  MU_CHECK_FLOAT(3.0, value[2]);

  return 0;
}

int test_Parse_parseVec2() {
  Vec2 value = parseVec2("X: 1.0 2.0");

  MU_CHECK_FLOAT(1.0, value(0));
  MU_CHECK_FLOAT(2.0, value(1));

  return 0;
}

int test_Parse_parseVec3() {
  Vec3 value = parseVec3("X: 1.0 2.0 3.0");

  MU_CHECK_FLOAT(1.0, value(0));
  MU_CHECK_FLOAT(2.0, value(1));
  MU_CHECK_FLOAT(3.0, value(2));

  return 0;
}

int test_Parse_parseVecX() {
  VecX value = parseVecX("X: 1.0 2.0 3.0 4.0 5.0 6.0");

  MU_CHECK_FLOAT(1.0, value(0));
  MU_CHECK_FLOAT(2.0, value(1));
  MU_CHECK_FLOAT(3.0, value(2));
  MU_CHECK_FLOAT(4.0, value(3));
  MU_CHECK_FLOAT(5.0, value(4));
  MU_CHECK_FLOAT(6.0, value(5));

  return 0;
}

int test_Parse_parseMat3() {
  Mat3 value = parseMat3("X: 1 2 3 4 5 6 7 8 9");

  MU_CHECK_FLOAT(1.0, value(0, 0));
  MU_CHECK_FLOAT(2.0, value(0, 1));
  MU_CHECK_FLOAT(3.0, value(0, 2));
  MU_CHECK_FLOAT(4.0, value(1, 0));
  MU_CHECK_FLOAT(5.0, value(1, 1));
  MU_CHECK_FLOAT(6.0, value(1, 2));
  MU_CHECK_FLOAT(7.0, value(2, 0));
  MU_CHECK_FLOAT(8.0, value(2, 1));
  MU_CHECK_FLOAT(9.0, value(2, 2));

  return 0;
}

int test_Parse_parseMat34() {
  Mat34 value = parseMat34("X: 1 2 3 4 5 6 7 8 9 10 11 12");

  MU_CHECK_FLOAT(1.0, value(0, 0));
  MU_CHECK_FLOAT(2.0, value(0, 1));
  MU_CHECK_FLOAT(3.0, value(0, 2));
  MU_CHECK_FLOAT(4.0, value(0, 3));

  MU_CHECK_FLOAT(5.0, value(1, 0));
  MU_CHECK_FLOAT(6.0, value(1, 1));
  MU_CHECK_FLOAT(7.0, value(1, 2));
  MU_CHECK_FLOAT(8.0, value(1, 3));

  MU_CHECK_FLOAT(9.0, value(2, 0));
  MU_CHECK_FLOAT(10.0, value(2, 1));
  MU_CHECK_FLOAT(11.0, value(2, 2));
  MU_CHECK_FLOAT(12.0, value(2, 3));

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_Parse_parseString);
  MU_ADD_TEST(test_Parse_parseDouble);
  MU_ADD_TEST(test_Parse_parseArray);
  MU_ADD_TEST(test_Parse_parseVec2);
  MU_ADD_TEST(test_Parse_parseVec3);
  MU_ADD_TEST(test_Parse_parseVecX);
  MU_ADD_TEST(test_Parse_parseMat3);
  MU_ADD_TEST(test_Parse_parseMat34);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
