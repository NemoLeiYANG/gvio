#ifndef GVIO_UTIL_UTIL_HPP
#define GVIO_UTIL_UTIL_HPP
/**
 * @file util.hpp
 */

#include "gvio/util/config.hpp"
#include "gvio/util/data.hpp"
#include "gvio/util/file.hpp"
#include "gvio/util/gps.hpp"
#include "gvio/util/linalg.hpp"
#include "gvio/util/log.hpp"
#include "gvio/util/math.hpp"
#include "gvio/util/opencv.hpp"
#include "gvio/util/stats.hpp"
#include "gvio/util/time.hpp"

// MACROS
#define UNUSED(expr)                                                           \
  do {                                                                         \
    (void) (expr);                                                             \
  } while (0)

// MACROS
#define CHECK(A, M, ...)                                                       \
  if (!(A)) {                                                                  \
    LOG_ERROR(M, ##__VA_ARGS__);                                               \
    goto error;                                                                \
  }

#endif
