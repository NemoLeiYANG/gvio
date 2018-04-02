#include "gvio/sim/bezier.hpp"

namespace gvio {

template <typename T>
T bezier(const std::vector<T> &points, const float t) {
  const int n = points.size() - 1;
  T result = zeros(3, 1);

  for (int i = 0; i <= n; i++) {
    const double binomial_term = binomial(n, i);
    const double polynomial_term = pow(1 - t, n - i) * pow(t, i);
    const T weight = points[i];
    result = result + binomial_term * polynomial_term * weight;
  }

  return result;
}

template <typename T>
T bezier_derivative(const std::vector<T> &points,
                    const float t,
                    const int order) {
  if (order == 0) {
    return bezier(points, t);
  }

  const int n = points.size() - 1;
  const int k = n - 1;
  std::vector<T> new_points;

  for (int i = 0; i <= k; i++) {
    const T derivative_weight = n * (points[i + 1] - points[i]);
    new_points.emplace_back(derivative_weight);
  }

  return bezier_derivative(new_points, t, order - 1);
}

template <typename T>
T bezier_tangent(const std::vector<T> &points, const float t) {
  const T point = bezier_derivative(points, t, 1);
  const double d = point.norm();
  const T tangent = point / d;

  return tangent;
}

template <typename T>
T decasteljau(const std::vector<T> &points, const float t) {
  if (points.size() == 1) {
    return points[0];
  }

  std::vector<T> new_points;
  for (int i = 0; i < points.size() - 1; i++) {
    new_points.push_back((1 - t) * points[i] + t * points[i + 1]);
  }

  return decasteljau(new_points, t);
}

} // namespace gvio
