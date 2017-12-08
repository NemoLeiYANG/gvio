#include "gvio/util/file.hpp"

namespace gvio {

bool file_exists(const std::string &path) {
  FILE *file;

  file = fopen(path.c_str(), "r");
  if (file != NULL) {
    fclose(file);
    return true;
  } else {
    return false;
  }
}

bool dir_exists(const std::string &path) {
  DIR *dir = opendir(path.c_str());

  if (dir) {
    closedir(dir);
    return true;
  } else if (ENOENT == errno) {
    return false;
  } else {
    LOG_ERROR("dir_exists() failed! %s", strerror(errno));
    exit(-1);
  }
}

std::string strip(const std::string &s, const std::string &target) {
  size_t first = s.find_first_not_of(target);
  if (std::string::npos == first) {
    return s;
  }

  size_t last = s.find_last_not_of(target);
  return s.substr(first, (last - first + 1));
}

std::string strip_end(const std::string &s, const std::string &target) {
  size_t last = s.find_last_not_of(target);
  return s.substr(0, last + 1);
}

int remove_dir(const std::string &path) {
  DIR *dir = opendir(path.c_str());
  struct dirent *next_file;
  char filepath[256];

  // pre-check
  if (dir == NULL) {
    // log_error("Failed to rmdir [%s]: %s", path.c_str(),
    // std::strerror(errno));
    return -1;
  }

  // remove files in path
  while ((next_file = readdir(dir)) != NULL) {
    // build the path for each file in the folder
    sprintf(filepath, "%s/%s", path.c_str(), next_file->d_name);
    remove(filepath);
  }

  // remove dir
  remove(path.c_str());
  closedir(dir);

  return 0;
}

std::vector<std::string> path_split(const std::string path) {
  std::string s;
  std::vector<std::string> splits;

  s = "";
  for (size_t i = 0; i < path.length(); i++) {
    if (s != "" && path[i] == '/') {
      splits.push_back(s);
      s = "";
    } else if (path[i] != '/') {
      s += path[i];
    }
  }
  splits.push_back(s);

  return splits;
}

void paths_combine(const std::string path1,
                   const std::string path2,
                   std::string &out) {
  int dirs_up;
  std::vector<std::string> splits1;
  std::vector<std::string> splits2;

  // setup
  out = "";
  splits1 = path_split(path1);
  splits2 = path_split(path2);

  // obtain number of directory ups in path 2
  dirs_up = 0;
  for (size_t i = 0; i < splits2.size(); i++) {
    if (splits2[i] == "..") {
      dirs_up++;
    }
  }

  // drop path1 elements as path2 dir ups
  for (int i = 0; i < dirs_up; i++) {
    splits1.pop_back();
  }

  // append path1 to out
  if (path1[0] == '/') {
    out += "/";
  }
  for (size_t i = 0; i < splits1.size(); i++) {
    out += splits1[i];
    out += "/";
  }

  // append path2 to out
  for (size_t i = dirs_up; i < splits2.size(); i++) {
    out += splits2[i];
    out += "/";
  }

  // remove trailing slashes
  for (size_t i = out.length() - 1; i > 0; i--) {
    if (out[i] == '/') {
      out.pop_back();
    } else {
      break;
    }
  }
}

} // end of gvio namepsace
