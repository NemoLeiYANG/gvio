#include "gvio/gvio_test.hpp"
#include "gvio/util/file.hpp"

namespace gvio {

TEST(file, file_exists) {
  EXPECT_TRUE(file_exists("tests/configs/control/position_controller.yaml"));
  EXPECT_FALSE(file_exists("tests/configs/control/bogus.yaml"));
}

TEST(file, path_split) {
  std::vector<std::string> splits;

  splits = path_split("/a/b/c.yaml");
  EXPECT_EQ(3, splits.size());
  EXPECT_EQ("a", splits[0]);
  EXPECT_EQ("b", splits[1]);
  EXPECT_EQ("c.yaml", splits[2]);
}

TEST(file, paths_combine) {
  std::string out;

  paths_combine("/a/b/c", "../", out);
  std::cout << out << std::endl;
  EXPECT_EQ("/a/b", out);

  paths_combine("/a/b/c", "../..", out);
  std::cout << out << std::endl;
  EXPECT_EQ("/a", out);

  paths_combine("/a/b/c", "d/e", out);
  std::cout << out << std::endl;
  EXPECT_EQ("/a/b/c/d/e", out);

  paths_combine("./a/b/c", "../d/e", out);
  std::cout << out << std::endl;
  EXPECT_EQ("./a/b/d/e", out);
}

} // namespace gvio
