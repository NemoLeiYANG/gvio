#ifndef GVIO_MSCKF_BLACKBOX_HPP
#define GVIO_MSCKF_BLACKBOX_HPP

#include <string>
#include <iostream>
#include <fstream>

#include "gvio/util/util.hpp"
#include "gvio/msckf/msckf.hpp"

namespace gvio {

/**
 * BlackBox
 */
class BlackBox {
public:
  std::ofstream output_file;

  BlackBox();
  virtual ~BlackBox();

  int configure(const std::string &output_path);
  int record(const MSCKF &msckf);
};

} // namespace gvio
#endif
