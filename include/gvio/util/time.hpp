#ifndef GVIO_UTILS_TIME_HPP
#define GVIO_UTILS_TIME_HPP
/**
 * @file time.hpp
 */

#include <sys/time.h>
#include <time.h>

namespace gvio {

void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);

/**
 * Get time now in milliseconds since epoch
 */
double time_now();

} // namespace gvio
#endif
