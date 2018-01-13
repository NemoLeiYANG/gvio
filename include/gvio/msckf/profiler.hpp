/**
 * @file
 * @ingroup msckf
 */
#ifndef GVIO_MSCKF_PROFILER_HPP
#define GVIO_MSCKF_PROFILER_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <map>

#include "gvio/util/util.hpp"
#include "gvio/msckf/msckf.hpp"

namespace gvio {
/**
 * @addtogroup msckf
 * @{
 */

class Profiler {
public:
  std::ofstream profile_file;
  std::map<std::string, struct timespec> timers;
  std::map<std::string, double> timings;

  Profiler();
  virtual ~Profiler();

  /**
   * Configure
   *
   * @param output_path Output path
   * @returns 0 for success, -1 for failure
   */
  int configure(const std::string &output_file);

  /**
   * Start profiling
   *
   * @param name Name of what we're profiling
   */
  int start(const std::string &name);

  /**
   * Stop profiling
   *
   * @param name Name of what we're profiling
   */
  int stop(const std::string &name);
};

/** @} group msckf */
} // namespace gvio
#endif // GVIO_MSCKF_PROFILER_HPP
