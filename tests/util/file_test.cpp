#include "gvio/munit.hpp"
#include "gvio/util/file.hpp"

namespace gvio {

int test_file_exists() {
  MU_CHECK(file_exists("test_configs/control/position_controller.yaml"));
  MU_FALSE(file_exists("test_configs/control/bogus.yaml"));

  return 0;
}

int test_path_split() {
  std::vector<std::string> splits;

  splits = path_split("/a/b/c.yaml");
  MU_CHECK_EQ(3, (int) splits.size());
  MU_CHECK_EQ("a", splits[0]);
  MU_CHECK_EQ("b", splits[1]);
  MU_CHECK_EQ("c.yaml", splits[2]);

  return 0;
}

int test_paths_combine() {
  std::string out;

  paths_combine("/a/b/c", "../", out);
  std::cout << out << std::endl;
  MU_CHECK_EQ("/a/b", out);

  paths_combine("/a/b/c", "../..", out);
  std::cout << out << std::endl;
  MU_CHECK_EQ("/a", out);

  paths_combine("/a/b/c", "d/e", out);
  std::cout << out << std::endl;
  MU_CHECK_EQ("/a/b/c/d/e", out);

  paths_combine("./a/b/c", "../d/e", out);
  std::cout << out << std::endl;
  MU_CHECK_EQ("./a/b/d/e", out);

  return 0;
}

void test_suite() {
  MU_ADD_TEST(test_file_exists);
  MU_ADD_TEST(test_path_split);
  MU_ADD_TEST(test_paths_combine);
}

} // namespace gvio

MU_RUN_TESTS(gvio::test_suite);
