#include "gvio/msckf/profiler.hpp"

namespace gvio {

Profiler::Profiler() {}

Profiler::~Profiler() {
  if (this->profile_file.good()) {
    this->profile_file.close();
  }
}

int Profiler::configure(const std::string &output_path) {
  this->profile_file.open(output_path);
  if (this->profile_file.good() == false) {
    LOG_ERROR("Failed to open profile file for recording [%s]",
              output_path.c_str());
    return -1;
  }

  return 0;
}

int Profiler::start(const std::string &name) {
  // Pre-check
  if (this->timers.find(name) != this->timers.end()) {
    LOG_ERROR("Timer already exists for [%s]!", name.c_str());
    return -1;
  } else if (this->timings.find(name) != this->timings.end()) {
    LOG_ERROR("Profile for [%s] already exists!", name.c_str());
    return -1;
  }

  // Add timer
  this->timers[name] = tic();

  return 0;
}

int Profiler::stop(const std::string &name) {
  // Pre-check
  if (this->timers.find(name) == this->timers.end()) {
    LOG_ERROR("No timer exists for [%s]!", name.c_str());
    return -1;
  } else if (this->timings.find(name) != this->timings.end()) {
    LOG_ERROR("Profile for [%s] already exists!", name.c_str());
    return -1;
  }

  // Stop timer
  const double time = toc(&this->timers[name]);
  this->timers.erase(this->timers.find(name));
  this->timings[name] = time;

  return 0;
}

} // namespace gvio
