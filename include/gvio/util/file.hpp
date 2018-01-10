/**
 * @file
 * @defgroup file file
 * @ingroup util
 */
#ifndef GVIO_UTIL_FILE_HPP
#define GVIO_UTIL_FILE_HPP

#include <dirent.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include "gvio/util/log.hpp"

namespace gvio {
/**
 * @addtogroup file
 * @{
 */

/**
 * Check if file exists
 *
 * @param path Path to file
 * @returns true or false
 */
bool file_exists(const std::string &path);

/**
 * Check if path exists
 *
 * @param path Path
 * @returns true or false
 */
bool dir_exists(const std::string &path);

/**
 * Strips a target character from the start and end of a string
 *
 * @param s String to strip
 * @param target Target character to strip
 * @returns Stripped string
 */
std::string strip(const std::string &s, const std::string &target = " ");

/**
 * Strips a target character from the end of a string
 *
 * @param s String to strip
 * @param target Target character to strip
 * @returns Stripped string
 */
std::string strip_end(const std::string &s, const std::string &target = " ");

/**
 * Create directory
 *
 * @param path Path to directory
 * @returns 0 for success, -1 for failure
 */
int create_dir(const std::string &path);

/**
 * Remove directory
 *
 * @param path Path to directory
 * @returns 0 for success, -1 for failure
 */
int remove_dir(const std::string &path);

/**
 * List directory
 *
 * @param path Path to directory
 * @param results List of files and directories
 * @returns 0 for success, -1 for failure
 */
int list_dir(const std::string &path, std::vector<std::string> &results);

/**
 * Split path into a number of elements
 *
 * @param path Path
 * @returns List of path elements
 */
std::vector<std::string> path_split(const std::string path);

/**
 * Combine `path1` and `path2`
 *
 * @param path1 Path 1
 * @param path2 Path 22
 * @param out Combined path
 */
void paths_combine(const std::string path1,
                   const std::string path2,
                   std::string &out);

/** @} group file */
} // end of gvio namepsace
#endif // GVIO_UTIL_FILE_HPP
