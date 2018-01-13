#include "gvio/munit.h"
#include "gvio/msckf/profiler.hpp"

namespace gvio {

int test_Profiler_constructor() {
  Profiler profiler;
  return 0;
}

int test_Profiler_configure() {
  Profiler profiler;

  const int retval = profiler.configure("/tmp/profile.dat");
  MU_CHECK_EQ(0, retval);

  return 0;
}

int test_Profiler_start_stop() {
  Profiler profiler;

  // Configure
  int retval = profiler.configure("/tmp/profile.dat");
  MU_CHECK_EQ(0, retval);

  // Profile start and stop
  retval = profiler.start("test");
  MU_CHECK_EQ(0, retval);

  retval = profiler.stop("test");
  MU_CHECK_EQ(0, retval);

  // Assert
  MU_CHECK_EQ(0, profiler.timers.size());
  MU_CHECK_EQ(1, profiler.timings.size());

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_Profiler_constructor);
  MU_ADD_TEST(test_Profiler_configure);
  MU_ADD_TEST(test_Profiler_start_stop);
}

} // namespace gvio
MU_RUN_TESTS(gvio::test_suite);
