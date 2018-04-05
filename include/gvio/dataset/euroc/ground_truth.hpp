/**
 * @file
 * @defgroup euroc euroc
 * @ingroup dataset
 */
#ifndef GVIO_DATASET_EUROC_GROUND_TRUTH_HPP
#define GVIO_DATASET_EUROC_GROUND_TRUTH_HPP

#include <vector>
#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup euroc
 * @{
 */

/**
 * Ground truth data
 */
struct GroundTruth {
  // Data
  std::vector<long> timestamps;
  std::vector<double> time;
  std::vector<Vec3> p_RS_R;
  std::vector<Vec4> q_RS;
  std::vector<Vec3> v_RS_R;
  std::vector<Vec3> b_w_RS_S;
  std::vector<Vec3> b_a_RS_S;

  /**
   * Load ground truth data
   *
   * @param data_dir Ground truth data directory
   * @returns 0 for success, -1 for failure
   */
  int load(const std::string &data_dir);
};

/** @} group euroc */
} // namespace gvio
#endif // GVIO_DATASET_EUROC_GROUND_TRUTH_HPP
