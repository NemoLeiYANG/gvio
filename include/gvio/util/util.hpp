/**
 * @file
 * @defgroup util util
 */
#ifndef GVIO_UTIL_UTIL_HPP
#define GVIO_UTIL_UTIL_HPP

#include "gvio/util/config.hpp"
#include "gvio/util/data.hpp"
#include "gvio/util/euler.hpp"
#include "gvio/util/file.hpp"
#include "gvio/util/gps.hpp"
#include "gvio/util/linalg.hpp"
#include "gvio/util/log.hpp"
#include "gvio/util/math.hpp"
#include "gvio/util/stats.hpp"
#include "gvio/util/time.hpp"
#include "gvio/util/vision.hpp"

namespace gvio {
/**
 * @addtogroup util
 * @{
 */

// MACROS
#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

// MACROS
#ifndef CHECK
#define CHECK(A, M, ...)                                                       \
  if (!(A)) {                                                                  \
    LOG_ERROR(M, ##__VA_ARGS__);                                               \
    goto error;                                                                \
  }
#endif

/** @} group util */
} // namespace gvio
#endif // GVIO_UTIL_UTIL_HPP
