/**
 * @file
 * @defgroup time time
 * @ingroup util
 */
#ifndef GVIO_UTILS_TIME_HPP
#define GVIO_UTILS_TIME_HPP

#include <sys/time.h>
#include <time.h>

namespace gvio {
/**
 * @addtogroup time
 * @{
 */

void tic(struct timespec *tic);
float toc(struct timespec *tic);
float mtoc(struct timespec *tic);

/**
 * Get time now in milliseconds since epoch
 */
double time_now();

/** @} group time */
} // namespace gvio
#endif
