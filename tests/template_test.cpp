#include "gvio/munit.hpp"

namespace gvio {

int test_hello() { return 0; }

void test_suite() { MU_ADD_TEST(test_hello); }

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
