#include "gvio/munit.h"
#include "gvio/util/data.hpp"

#define TEST_DATA "test_data/utils/matrix.dat"
#define TEST_OUTPUT "/tmp/matrix.dat"

namespace gvio {

int test_csvrows() {
  int rows;
  rows = csvrows(TEST_DATA);
  MU_CHECK_EQ(281, rows);
  return 0;
}

int test_csvcols() {
  int cols;
  cols = csvcols(TEST_DATA);
  MU_CHECK_EQ(2, cols);
  return 0;
}

int test_csv2mat() {
  MatX data;

  csv2mat(TEST_DATA, true, data);
  MU_CHECK_EQ(280, data.rows());
  MU_CHECK_EQ(2, data.cols());
  MU_CHECK_FLOAT(-2.22482078596, data(0, 0));
  MU_CHECK_FLOAT(9.9625789766, data(0, 1));
  MU_CHECK_FLOAT(47.0485650525, data(279, 0));
  MU_CHECK_FLOAT(613.503760567, data(279, 1));

  return 0;
}

int test_mat2csv() {
  MatX x;
  MatX y;

  csv2mat(TEST_DATA, true, x);
  mat2csv(TEST_OUTPUT, x);
  csv2mat(TEST_OUTPUT, false, y);

  for (int i = 0; i < x.rows(); i++) {
    for (int j = 0; j < x.cols(); j++) {
      MU_CHECK_NEAR(x(i, j), y(i, j), 0.1);
    }
  }

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_csvrows);
  MU_ADD_TEST(test_csvcols);
  MU_ADD_TEST(test_csv2mat);
  MU_ADD_TEST(test_mat2csv);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
