#ifndef GVIO_UTILS_CONFIG_HPP
#define GVIO_UTILS_CONFIG_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>

#include <yaml-cpp/yaml.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "gvio/util/file.hpp"
#include "gvio/util/log.hpp"
#include "gvio/util/math.hpp"

namespace gvio {

/**
 * An enum used by `ConfigParam` to denote the yaml value type.
 *
 *  Currently we support parsing of the following data types:
 *  - **Primitives**: `bool`, `int`, `float`, `double`, `std::string`
 *  - **Arrays**: `std::vector<bool>`, `std::vector<int>`,
 * `std::vector<float>`,
 * `std::vector<double>`, `std::vector<std::string>`
 *  - **Vectors**: `Eigen::Vector2d`, `Eigen::Vector3d`, `Eigen::Vector4d`,
 * `Eigen::VectorXd`
 *  - **Matrices**: `Eigen::Matrix2d`, `Eigen::Matrix3d`, `Eigen::Matrix4d`,
 * `Eigen::MatrixXd`
 */
enum ConfigDataType {
  TYPE_NOT_SET = 0,
  // PRIMITIVES
  BOOL = 1,
  INT = 2,
  FLOAT = 3,
  DOUBLE = 4,
  STRING = 5,
  // ARRAY
  BOOL_ARRAY = 11,
  INT_ARRAY = 12,
  FLOAT_ARRAY = 13,
  DOUBLE_ARRAY = 14,
  STRING_ARRAY = 15,
  // VECTOR
  VEC2 = 21,
  VEC3 = 22,
  VEC4 = 23,
  VECX = 24,
  // MATRIX
  MAT2 = 31,
  MAT3 = 32,
  MAT4 = 33,
  MATX = 34,
  CVMAT = 35
};

/** Represents a parameter to be parsed in the yaml file */
class ConfigParam {
public:
  enum ConfigDataType type = TYPE_NOT_SET; //!< parameter type (e.g `INT`)
  std::string key;                         //!< yaml key to parse from
  void *data = nullptr; //!< pointer to variable to store the parameter value
  bool optional = false;

  ConfigParam() {}

  ConfigParam(ConfigDataType type, std::string key, void *out, bool optional)
      : type{type}, key{key}, data{out}, optional{optional} {};
};

/**
 * Parses yaml files for parameters of different types.
 *
 * Currently `ConfigParser` supports parsing the following data types:
 * - **Primitives**: `bool`, `int`, `float`, `double`, `std::string`
 * - **Arrays**: `std::vector<bool>`, `std::vector<int>`,
 * `std::vector<float>`,
 * `std::vector<double>`, `std::vector<std::string>`
 * - **Vectors**: `Eigen::Vector2d`, `Eigen::Vector3d`, `Eigen::Vector4d`,
 * `Eigen::VectorXd`
 * - **Matrices**: `Eigen::Matrix2d`, `Eigen::Matrix3d`, `Eigen::Matrix4d`,
 * `Eigen::MatrixXd`
 *
 * @note For a **matrix** we require the yaml to have three nested keys in
 * order
 * for `ConfigParser` to operate properly. They are `rows`, `cols` and `data`.
 * For example:
 * ```yaml
 * some_matrix:
 *   rows: 3
 *   cols: 3
 *   data: [1.1, 2.2, 3.3,
 *          4.4, 5.5, 6.6,
 *          7.7, 8.8, 9.9]
 * ```
 *
 * For example, consider this yaml file:
 * @include example_params.yaml
 *
 * The code that parses the above is:
 * @include example_config_parser.cpp
 *
 * @todo integrate unit tests into examples (see
 * http://stackoverflow.com/a/16034375/431033)
 */
class ConfigParser {
public:
  bool config_loaded;

  std::string file_path;
  YAML::Node root;
  std::vector<ConfigParam> params;

  /**
   * Default constructor. By default it sets:
   *
   * - `config_loaded` to `false`
   *
   * This variable is set to `true` once the yaml file is loaded.
   */
  ConfigParser();

  /**
   * Use the variations of `addParam` to add parameters you would like to
   * parse from the yaml file, where `key` is the yaml key, `out` is
   * dependent on the type of parameter you want to parse to and an
   * `optional` parameter to define whether `ConfigParser` should fail if the
   * parameter is not found.
   */
  // clang-format off
  void addParam(const std::string &key, bool *out, const bool optional = false);
  void addParam(const std::string &key, int *out, const bool optional = false);
  void addParam(const std::string &key, float *out, const bool optional = false);
  void addParam(const std::string &key, double *out, const bool optional = false);
  void addParam(const std::string &key, std::string *out, const bool optional = false);
  void addParam(const std::string &key, std::vector<bool> *out, const bool optional = false);
  void addParam(const std::string &key, std::vector<int> *out, const bool optional = false);
  void addParam(const std::string &key, std::vector<float> *out, const bool optional = false);
  void addParam(const std::string &key, std::vector<double> *out, const bool optional = false);
  void addParam(const std::string &key, std::vector<std::string> *out, const bool optional = false);
  void addParam(const std::string &key, Vec2 *out, const bool optional = false);
  void addParam(const std::string &key, Vec3 *out, const bool optional = false);
  void addParam(const std::string &key, Vec4 *out, const bool optional = false);
  void addParam(const std::string &key, VecX *out, const bool optional = false);
  void addParam(const std::string &key, Mat2 *out, const bool optional = false);
  void addParam(const std::string &key, Mat3 *out, const bool optional = false);
  void addParam(const std::string &key, Mat4 *out, const bool optional = false);
  void addParam(const std::string &key, MatX *out, const bool optional = false);
  void addParam(const std::string &key, cv::Mat *out, const bool optional = false);
  // clang-format on

  /**
   * Get yaml node given yaml `key`. The result is assigned to `node` if
   * `key` matches anything in the config file, else `node` is set to `NULL`.
   */
  int getYamlNode(const std::string &key, YAML::Node &node);

  /**
   * @return a status code meaning
   *
    * - `0`: On success
    * - `-1`: Config file is not loaded
    * - `-2`: `key` not found in yaml file, parameter is not optional
    * - `-3`: `key` not found in yaml file, parameter is optional
    * - `-4`: Invalid vector (wrong size)
    * - `-5`: Invalid matrix (missing yaml key 'rows', 'cols' or 'data' for
    * matrix)
    */
  /** @return see `getYamlNode` */
  int checkKey(const std::string &key, bool optional);

  /**
   * @return see `checkKey`
   *
   * @todo refactor return codes into an enum which can be documented
   */
  int checkVector(const std::string &key,
                  enum ConfigDataType type,
                  bool optional);

  /** @return see `checkKey` */
  int checkMatrix(const std::string &key, bool optional);

  /**
   * Load yaml param primitive, array, vector or matrix.
   *
   * @return
   * - `0`: On success
   * - `-1`: Config file is not loaded
   * - `-2`: `key` not found in yaml file, parameter is not optional
   * - `-3`: `key` not found in yaml file, parameter is optional
   * - `-4`: Invalid vector (wrong size)
   * - `-5`: Invalid matrix (missing yaml key 'rows', 'cols' or 'data' for
   * matrix)
   * - `-6`: Invalid param type
   */
  int loadPrimitive(ConfigParam &param);
  int loadArray(ConfigParam &param);
  int loadVector(ConfigParam &param);
  int loadMatrix(ConfigParam &param);

  /**
   * Load yaml file at `config_file`.
   *
   * @return
   * - `0`: On success
   * - `1`: File not found
   * - `-1`: Config file is not loaded
   * - `-2`: `key` not found in yaml file
   * - `-4`: Invalid vector (wrong size)
   * - `-5`: Invalid matrix (missing yaml key 'rows', 'cols' or 'data' for
   * matrix)
   * - `-6`: Invalid param type
   */
  int load(const std::string &config_file);
};

} // namespace atl

#endif // GVIO_UTILS_CONFIG_HPP
