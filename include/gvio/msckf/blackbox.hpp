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
  std::ofstream win_file;

  BlackBox();
  virtual ~BlackBox();

  /**
   * Configure
   *
   * @param output_path Output path
   * @param base_name Output file basename
   *
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &output_path, const std::string &base_name);

  /**
   * Record ground truth
   *
   * @param time Relative time in seconds (where 0 is start)
   * @param msckf MSCKF
   *
   * @returns 0 for success, -1 for failure
   */
  int recordEstimate(const double time, const MSCKF &msckf);

  /**
   * Record measurement
   *
   * @param time Relative time in seconds (where 0 is start)
   * @param measurement_a_B Accelerometer measurement
   * @param measurement_w_B Gyroscope measurement
   *
   * @returns 0 for success, -1 for failure
   */
  int recordMeasurement(const double time,
                        const Vec3 &measurement_a_B,
                        const Vec3 &measurement_w_B);

  /**
   * Record ground truth
   *
   * @param time Relative time in seconds (where 0 is start)
   * @param ground_truth_p_G Ground Truth position in global frame
   * @param ground_truth_v_G Ground Truth velocity in global frame
   * @param ground_truth_rpy_G Ground Truth roll, pitch, and yaw in global frame
   *
   * @returns 0 for success, -1 for failure
   */
  int recordGroundTruth(const double time,
                        const Vec3 &ground_truth_p_G,
                        const Vec3 &ground_truth_v_G,
                        const Vec3 &ground_truth_rpy_G);

  /**
   * Record MSCKF camera states
   *
   * @param msckf MSCKF
   * @returns 0 for success, -1 for failure
   */
  int recordCameraStates(const MSCKF &msckf);

  /**
   * Record time step
   *
   * @param time Relative time in seconds (where 0 is start)
   * @param msckf MSCKF
   * @param measurement_a_B Accelerometer measurement
   * @param measurement_w_B Gyroscope measurement
   * @param ground_truth_p_G Ground Truth position in global frame
   * @param ground_truth_v_G Ground Truth velocity in global frame
   * @param ground_truth_rpy_G Ground Truth roll, pitch, and yaw in global frame
   *
   * @returns 0 for success, -1 for failure
   */
  int recordTimeStep(const double time,
                     const MSCKF &msckf,
                     const Vec3 &measurement_a_B,
                     const Vec3 &measurement_w_B,
                     const Vec3 &ground_truth_p_G,
                     const Vec3 &ground_truth_v_G,
                     const Vec3 &ground_truth_rpy_G);
};

} // namespace gvio
#endif
