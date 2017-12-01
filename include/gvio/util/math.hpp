#ifndef GVIO_UTILS_MATH_HPP
#define GVIO_UTILS_MATH_HPP

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "gvio/util/log.hpp"

namespace gvio {

#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__
typedef Eigen::Vector2d Vec2;
typedef Eigen::Vector3d Vec3;
typedef Eigen::Vector4d Vec4;
typedef Eigen::VectorXd VecX;

typedef Eigen::Matrix2d Mat2;
typedef Eigen::Matrix3d Mat3;
typedef Eigen::Matrix4d Mat4;
typedef Eigen::MatrixXd MatX;

typedef Eigen::Matrix<double, 3, 4> Mat34;

typedef Eigen::Quaterniond Quaternion;
#endif

/**
 * Create random integer
 * @param ub Upper bound
 * @param lb Lower bound
 * @return Random integer
 */
int randi(const int ub, const int lb);

/**
 * Create random double
 * @param ub Upper bound
 * @param lb Lower bound
 * @return Random floating point
 */
double randf(const double ub, const double lb);

/**
 * Sign of number
 * @param x Number to check sign
 * @return
 *    - 0: Number is zero
 *    - 1: Positive number
 *    - -1: Negative number
 */
int sign(const double x);

/**
 * Floating point comparator
 * @param f1 First value
 * @param f2 Second value
 * @return
 *    - 0: if equal
 *    - 1: if f1 > f2
 *    - -1: if f1 < f2
 */
int fltcmp(const double f1, const double f2);

/**
 * Calculate median given an array of numbers
 * @param v Array of numbers
 * @return Median of given array
 */
double median(const std::vector<double> &v);

/**
 * Degrees to radians
 * @param d Degree to be converted
 * @return Degree in radians
 */
double deg2rad(const double d);

/**
 * Radians to degree
 * @param r Radian to be converted
 * @return Radian in degrees
 */
double rad2deg(const double r);

/**
 * Load std::vector of doubles to an Eigen::Matrix
 * @param x Matrix values
 * @param rows Number of matrix rows
 * @param cols Number of matrix colums
 * @param y Output matrix
 */
void load_matrix(const std::vector<double> &x,
                 const int rows,
                 const int cols,
                 MatX &y);

/**
 * Load an Eigen::Matrix into a std::vector of doubles
 * @param A Matrix
 * @param x Output vector of matrix values
 */
void load_matrix(const MatX A, std::vector<double> &x);

/**
 * Wrap angle in degrees to 180
 *
 * @param d Degrees
 * @return Angle wraped to 180
 */
double wrapTo180(const double d);

/**
 * Wrap angle in degrees to 360
 *
 * @param d Degrees
 * @return Angle wraped to 360
 */
double wrapTo360(const double d);

/**
 * Wrap angle in radians to PI
 *
 * @param r Radians
 * @return Angle wraped to PI
 */
double wrapToPi(const double r);

/**
 * Wrap angle in radians to 2 PI
 *
 * @param r Radians
 * @return Angle wraped to 2 PI
 */
double wrapTo2Pi(const double r);

/**
 * Cross-Track error based on waypoint line between p1, p2, and robot position
 *
 * @param p1 Waypoint 1
 * @param p2 Waypoint 2
 * @param pos Robot position
 * @return Cross track error
 */
double cross_track_error(const Vec2 &p1, const Vec2 &p2, const Vec2 &pos);

/**
 * Check if point `pos` is left or right of line formed by `p1` and `p2`
 *
 * @param p1 Waypoint 1
 * @param p2 Waypoint 2
 * @param pos Robot position
 * @return
 *    - 1: Point is left of waypoint line formed by `p1` and `p2`
 *    - 2: Point is right of waypoint line formed by `p1` and `p2`
 *    - 0: Point is colinear with waypoint line formed by `p1` and `p2`
 */
int point_left_right(const Vec2 &p1, const Vec2 &p2, const Vec2 &pos);

/**
 * Calculate closest point given waypoint line between `p1`, `p2` and robot
 * position
 *
 * @param p1 Waypoint 1
 * @param p2 Waypoint 2
 * @param pos Robot position
 * @param cloest Closest point
 * @return
 *    Unit number denoting where the closest point is on waypoint line. For
 *    example, a return value of 0.5 denotes the closest point is half-way
 *    (50%) of the waypoint line, alternatively a negative number denotes the
 *    closest point is behind the first waypoint.
 */
double
closest_point(const Vec2 &p1, const Vec2 &p2, const Vec2 &p3, Vec2 &closest);

/**
 * Linear interpolation between two points
 *
 * @param a First point
 * @param b Second point
 * @param mu Unit number
 */
Vec2 lerp(const Vec2 &a, const Vec2 &b, const double mu);

/**
 * Right hand rotation Matrix in x-axis
 * @param angle Rotation angle in radians
 * @return Rotation matrix
 */
Mat3 rotx(const double angle);

/**
 * Right hand rotation Matrix in y-axis
 * @param angle Rotation angle in radians
 * @return Rotation matrix
 */
Mat3 roty(const double angle);

/**
 * Right hand rotation Matrix in z-axis
 * @param angle Rotation angle in radians
 * @return Rotation matrix
 */
Mat3 rotz(const double angle);

/**
 * Convert Euler 1-2-3 angles to quaternion
 * @param euler Input Euler angles
 * @return Output quaternion
 */
Quaternion euler123ToQuat(const Vec3 &euler);

/**
 * Convert Euler 3-2-1 angles to quaternion
 * @param euler Input Euler angles
 * @return Output quaternion
 */
Quaternion euler321ToQuat(const Vec3 &euler);

/**
 * Convert Euler 1-2-3 angles to rotation matrix
 * @param euler Input Euler angles
 * @return R Output rotation matrix
 */
Mat3 euler123ToRot(const Vec3 &euler);

/**
 * Convert Euler 3-2-1 angles to rotation matrix
 * @param euler Input Euler angles
 * @return R Output rotation matrix
 */
Mat3 euler321ToRot(const Vec3 &euler);

/**
 * Convert quanternion to Euler 1-2-3 angles
 * @param q Input quaternion
 * @return euler Output Euler angles
 */
Vec3 quatToEuler123(const Quaternion &q);

/**
 * Convert quanternion to Euler 3-2-1 angles
 * @param q Input quaternion
 * @return euler Output Euler angles
 */
Vec3 quatToEuler321(const Quaternion &q);

/**
 * Convert Quaternion to rotation matrix
 * @param q Input quaternion
 * @param R Output rotation matrix
 */
Mat3 quat2rot(const Quaternion &q);

/**
 * Convert from ENU to NWU
 * @param enu ENU vector
 * @return nwu NWU vector
 */
Vec3 enu2nwu(const Vec3 &enu);

/**
 * Convert from EDN to NWU
 * @param enu EDN vector
 * @return nwu NWU vector
 */
Vec3 edn2nwu(const Vec3 &edn);

/**
 * Convert from EDN to ENU
 * @param enu EDN vector
 * @return enu ENU vector
 */
Vec3 edn2enu(const Vec3 &edn);

/**
 * Convert from NWU to ENU
 * @param nwu NWU vector
 * @return enu ENU vector
 */
Vec3 nwu2enu(const Vec3 &nwu);

/**
 * Convert from NWU to NED
 * @param nwu NWU vector
 * @return enu NED vector
 */
Vec3 nwu2ned(const Vec3 &nwu);

/**
 * Convert from NED to ENU
 * @param ned NED vector
 * @return enu ENU vector
 */
Vec3 ned2enu(const Vec3 &ned);

/**
 * Convert from NED to NWU
 * @param ned NED vector
 * @return enu NWU vector
 */
Vec3 ned2nwu(const Vec3 &ned);

/**
 * Convert from NWU to NED
 * @param nwu NWU quaternion
 * @return ned NED quaternion
 */
Quaternion nwu2ned(const Quaternion &nwu);

/**
 * Conver from NED to NWU
 * @param ned NED quaternion
 * @param nwu NWU quaternion
 */
Quaternion ned2nwu(const Quaternion &ned);

/**
 * Conver from ENU to NWU
 * @param ned ENU quaternion
 * @param nwu NWU quaternion
 */
Quaternion enu2nwu(const Quaternion &enu);

} // namespace gvio
#endif
