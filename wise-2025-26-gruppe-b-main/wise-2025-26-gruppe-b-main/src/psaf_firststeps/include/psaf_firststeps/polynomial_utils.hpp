#ifndef PSAF_FIRSTSTEPS__POLYNOMIAL_UTILS_HPP_
#define PSAF_FIRSTSTEPS__POLYNOMIAL_UTILS_HPP_

#include <array>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

struct QuadraticPolynomial
{
  double a{0.0};
  double b{0.0};
  double c{0.0};
};

inline bool isValidPoint(const geometry_msgs::msg::Point & p, double max_lateral)
{
  if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
    return false;
  }
  return std::abs(p.y) <= max_lateral;
}

inline std::optional<QuadraticPolynomial> fitQuadratic(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  if (points.size() < 3) {
    return std::nullopt;
  }

  double s_x = 0.0, s_x2 = 0.0, s_x3 = 0.0, s_x4 = 0.0;
  double s_y = 0.0, s_xy = 0.0, s_x2y = 0.0;
  const double n = static_cast<double>(points.size());

  for (const auto & p : points) {
    const double x = p.x;
    const double y = p.y;
    const double x2 = x * x;

    s_x += x;
    s_x2 += x2;
    s_x3 += x2 * x;
    s_x4 += x2 * x2;

    s_y += y;
    s_xy += x * y;
    s_x2y += x2 * y;
  }

  const double det = n * (s_x2 * s_x4 - s_x3 * s_x3) - s_x * (s_x * s_x4 - s_x2 * s_x3) +
    s_x2 * (s_x * s_x3 - s_x2 * s_x2);

  if (std::abs(det) < 1e-9) {
    return std::nullopt;
  }

  const double det_a = n * (s_xy * s_x4 - s_x3 * s_x2y) - s_x * (s_x * s_x4 - s_x2 * s_x3) +
    s_x2 * (s_x * s_x2y - s_xy * s_x2);
  const double det_b = n * (s_x2 * s_x2y - s_xy * s_x3) - s_x * (s_xy * s_x2 - s_y * s_x3) +
    s_x2 * (s_y * s_x2 - s_x * s_x2y);
  const double det_c = s_y * (s_x2 * s_x4 - s_x3 * s_x3) - s_x * (s_xy * s_x4 - s_x3 * s_x2y) +
    s_x2 * (s_xy * s_x3 - s_x2 * s_x2y);

  QuadraticPolynomial poly{};
  poly.a = det_a / det;
  poly.b = det_b / det;
  poly.c = det_c / det;

  return poly;
}

inline double curvatureAt(const QuadraticPolynomial & poly, double x)
{
  const double first = 2.0 * poly.a * x + poly.b;
  const double second = 2.0 * poly.a;
  const double denom = std::pow(1.0 + first * first, 1.5);
  if (denom < 1e-9) {
    return 0.0;
  }
  return second / denom;
}

#endif  // PSAF_FIRSTSTEPS__POLYNOMIAL_UTILS_HPP_
