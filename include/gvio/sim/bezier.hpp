/**
 * @file
 * @ingroup sim
 */
#ifndef GVIO_SIM_BEZIER_HPP
#define GVIO_SIM_BEZIER_HPP

#include <vector>

#include "gvio/util/util.hpp"

namespace gvio {
/**
 * @addtogroup sim
 * @{
 */

/**
 * Bezier curve of any arbitrary order
 * Source: https://pomax.github.io/bezierinfo/
 *
 * @param points Bezier curve control points in order
 * @param t Parameter
 * @returns Position on bezier curve at t
 */
template <typename T>
T bezier(const std::vector<T> &points, const float t);

/**
 * Derivative of an arbitrary order Bezier curve
 * Source: https://pomax.github.io/bezierinfo/#derivatives
 *
 * @param points Bezier curve control points in order
 * @param t Parameter
 * @returns Position on bezier curve at t
 */
template <typename T>
T bezier_derivative(const std::vector<T> &points,
                    const float t,
                    const int order);

/**
 * De Casteljau's algorithm
 * Source: https://pomax.github.io/bezierinfo/
 *
 * A recursive method to evaluate polynomials in Bernstein form or Bezier
 * curves. This algorithm can also be used to split a single Bezier curve into
 * two at an arbitrary parameter value.
 *
 * @param points Bezier curve control points in order
 * @param t Parameter
 * @returns Position on bezier curve at t
 */
template <typename T>
T decasteljau(const std::vector<T> &points, const float t);

/** @} group sim */
} // namespace gvio

#include "impl/bezier.hpp"

#endif // GVIO_SIM_BEZIER_HPP
