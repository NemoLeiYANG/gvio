#ifndef GVIO_UTILS_FILE_HPP
#define GVIO_UTILS_FILE_HPP

#include <dirent.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include "gvio/util/log.hpp"

namespace gvio {

bool file_exists(const std::string &path);
bool dir_exists(const std::string &path);
std::string strip(const std::string &s, const std::string &target = " ");
std::string strip_end(const std::string &s, const std::string &target = " ");
int remove_dir(const std::string &path);
std::vector<std::string> path_split(const std::string path);
void paths_combine(const std::string path1,
                   const std::string path2,
                   std::string &out);

} // end of gvio namepsace
#endif
