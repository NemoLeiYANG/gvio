#ifndef GVIO_UTILS_FILE_HPP
#define GVIO_UTILS_FILE_HPP

#include <dirent.h>
#include <stdio.h>

#include <iostream>
#include <numeric>
#include <string>
#include <vector>

namespace gvio {

bool file_exists(const std::string &name);
int remove_dir(const std::string &path);
std::vector<std::string> path_split(const std::string path);
void paths_combine(const std::string path1,
                   const std::string path2,
                   std::string &out);

} // end of gvio namepsace
#endif
