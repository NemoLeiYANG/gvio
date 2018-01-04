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
  std::ofstream est_file;
  std::ofstream mea_file;
  std::ofstream gnd_file;

  BlackBox();
  virtual ~BlackBox();

  int configure(const std::string &output_path, const std::string &base_name);
  int record(const double time,
             const MSCKF &msckf,
             const Vec3 &measurement_a_B,
             const Vec3 &measurement_w_B,
             const Vec3 &ground_truth_p_G,
             const Vec3 &ground_truth_v_G,
             const Vec3 &ground_truth_rpy_G);
};

} // namespace gvio
#endif
