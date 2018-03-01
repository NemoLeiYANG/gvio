#include "gvio/munit.hpp"

namespace gvio {

int test_constructor() { return 0; }
int test_load() { return 0; }

void test_suite() {
  MU_ADD_TEST(test_constructor);
  MU_ADD_TEST(test_load);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
