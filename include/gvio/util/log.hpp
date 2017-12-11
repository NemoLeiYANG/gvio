/**
 * @file
 * @defgroup log log
 * @ingroup util
 */
#ifndef GVIO_UTIL_LOG_HPP
#define GVIO_UTIL_LOG_HPP

namespace gvio {
/**
 * @addtogroup log
 * @{
 */

#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_ERROR(M, ...)                                                      \
  fprintf(stderr,                                                              \
          "[ERROR] [%s:%d] " M "\n",                                           \
          __FILENAME__,                                                        \
          __LINE__,                                                            \
          ##__VA_ARGS__)

#define LOG_INFO(M, ...) fprintf(stdout, "[INFO] " M "\n", ##__VA_ARGS__)

/** @} group log */
} // namespace gvio
#endif // GVIO_UTIL_LOG_HPP
