#pragma once
#include "voyage/cotila/cotila.h" // IWYU pragma: keep
#include "voyage/gcem/gcem.hpp"   // IWYU pragma: keep
#include <cstddef>
#include <cstdlib>
#include <optional>

// Implementation is a C++ constexpr version of:
// https://atsushisakai.github.io/PythonRobotics/modules/5_path_planning/cubic_spline/cubic_spline.html
// https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/CubicSpline/cubic_spline_planner.py#L146

namespace voyage {

template <std::size_t N> using vector = cotila::vector<float, N>;

template <std::size_t N, std::size_t M>
using matrix = cotila::matrix<float, N, M>;

template <std::size_t x_len>
constexpr matrix<x_len, x_len>
calc_matrix_a(const vector<x_len - 1> diffs) noexcept {
  auto res = cotila::identity<float, x_len>;

  for (std::size_t i = 0; i < x_len - 1; i++) {
    if (i != x_len - 2) {
      res[i + 1][i + 1] = 2.0 * (diffs[i] + diffs[i + 1]);
    }
    res[i + 1][i] = diffs[i];
    res[i][i + 1] = diffs[i];
  }

  res[0][1] = 0.0;
  res[x_len - 1][x_len - 2] = 0.0;
  res[x_len - 1][x_len - 1] = 1.0;

  return res;
}

template <std::size_t x_len>
constexpr matrix<x_len, 1>
calc_matrix_b(const vector<x_len - 1> diffs,
              const matrix<x_len, 1> coeff_a) noexcept {
  auto res = cotila::generate<x_len>([](int _) -> float { return 0.0; });

  for (std::size_t i = 0; i < x_len - 2; i++) {
    res[i + 1] = 3.0 * (coeff_a[i + 2][0] - coeff_a[i + 1][0]) / diffs[i + 1] -
                 3.0 * (coeff_a[i + 1][0] - coeff_a[i][0]) / diffs[i];
  }

  return cotila::as_column(res);
}

// Ax = B
// x = A^{-1}B
template <std::size_t x_len>
constexpr matrix<x_len, 1> linalg_solve(const matrix<x_len, x_len> a,
                                        const matrix<x_len, 1> b) noexcept {
  auto inverse = cotila::inverse(a);
  auto x = cotila::matmul(inverse, b);

  return x;
}

template <std::size_t x_len>
constexpr matrix<x_len, 1> calc_coeff_a(const vector<x_len> y) noexcept {

  return cotila::as_column(y);
}

template <std::size_t x_len>
constexpr matrix<x_len, 1>
calc_coeff_c(const vector<x_len - 1> diffs,
             const matrix<x_len, 1> coeff_a) noexcept {
  auto a = calc_matrix_a<x_len>(diffs);
  auto b = calc_matrix_b(diffs, coeff_a);

  return linalg_solve(a, b);
}

template <std::size_t x_len>
constexpr matrix<x_len, 1>
calc_coeff_b(const vector<x_len - 1> diffs, const matrix<x_len, 1> coeff_a,
             const matrix<x_len, 1> coeff_c) noexcept {
  auto res = cotila::generate<x_len>([](int _) -> float { return 0.0; });

  for (std::size_t i = 0; i < x_len - 1; i++) {
    res[i] = (1.0 / diffs[i]) * (coeff_a[i + 1][0] - coeff_a[i][0]) -
             (diffs[i] / 3.0) * (2.0 * coeff_c[i][0] + coeff_c[i + 1][0]);
  }

  return cotila::as_column(res);
}

template <std::size_t x_len>
constexpr matrix<x_len, 1>
calc_coeff_d(const vector<x_len - 1> diffs,
             const matrix<x_len, 1> coeff_c) noexcept {
  auto res = cotila::generate<x_len>([](int _) -> float { return 0.0; });

  for (std::size_t i = 0; i < x_len - 1; i++) {
    res[i] = (coeff_c[i + 1][0] - coeff_c[i][0]) / (3.0 * diffs[i]);
  }

  return cotila::as_column(res);
}

template <std::size_t x_len>
constexpr vector<x_len - 1> calc_diffs(const vector<x_len> x) noexcept {
  auto res = cotila::generate<x_len - 1>([](int _) -> float { return 0.0; });

  for (std::size_t i = 0; i < x_len - 1; i++) {
    res[i] = x[i + 1] - x[i];
  }

  return res;
}

template <std::size_t x_len>
constexpr std::size_t
find_segment_index(const float x, const vector<x_len> x_values) noexcept {
  std::size_t first = 0;
  std::size_t count = x_len;

  while (count > 0) {
    std::size_t step = count / 2;
    std::size_t curr = first + step;

    if (!(x < x_values[curr])) {
      first = curr + 1;
      count -= step + 1;
    } else {
      count = step;
    }
  }

  return static_cast<std::size_t>(std::max(static_cast<int>(first) - 1, 0));
}

template <std::size_t x_len>
constexpr std::optional<float>
calc_position(const float x, const vector<x_len> x_values,
              const matrix<x_len, 1> coeff_a, const matrix<x_len, 1> coeff_b,
              const matrix<x_len, 1> coeff_c,
              const matrix<x_len, 1> coeff_d) noexcept {
  if (x < x_values[0] || x > x_values[x_values.size - 1]) {
    return std::nullopt;
  }

  auto i = find_segment_index(x, x_values);
  auto dx = x - x_values[i];
  auto position = coeff_a[i][0] + coeff_b[i][0] * dx +
                  coeff_c[i][0] * gcem::pow(dx, 2.0) +
                  coeff_d[i][0] * gcem::pow(dx, 3.0);

  return position;
}

template <std::size_t x_len>
constexpr std::optional<float> calc_first_derivative(
    const float x, const vector<x_len> x_values, const matrix<x_len, 1> coeff_b,
    const matrix<x_len, 1> coeff_c, const matrix<x_len, 1> coeff_d) noexcept {
  if (x < x_values[0] || x > x_values[x_values.size - 1]) {
    return std::nullopt;
  }

  auto i = find_segment_index(x, x_values);
  auto dx = x - x_values[i];
  auto dy = coeff_b[i][0] + 2.0 * coeff_c[i][0] * dx +
            3.0 * coeff_d[i][0] * gcem::pow(dx, 2.0);

  return dy;
}

template <std::size_t x_len>
constexpr std::optional<float>
calc_second_derivative(const float x, const vector<x_len> x_values,
                       const matrix<x_len, 1> coeff_c,
                       const matrix<x_len, 1> coeff_d) noexcept {
  if (x < x_values[0] || x > x_values[x_values.size - 1]) {
    return std::nullopt;
  }

  auto i = find_segment_index(x, x_values);
  auto dx = x - x_values[i];
  auto ddy = 2.0 * coeff_c[i][0] + 6.0 * coeff_d[i][0] * dx;

  return ddy;
}

template <std::size_t x_len> class CubicSpline1d {
public:
  // x must be strictly monotone increasing
  constexpr CubicSpline1d(vector<x_len> x, vector<x_len> y)
      : x(x), y(y), diffs(calc_diffs(x)), coeff_a(calc_coeff_a(y)),
        coeff_c(calc_coeff_c(diffs, coeff_a)),
        coeff_b(calc_coeff_b(diffs, coeff_a, coeff_c)),
        coeff_d(calc_coeff_d(diffs, coeff_c)) {}

  constexpr std::optional<float> calc_pos(const float x) const {
    return calc_position(x, this->x, this->coeff_a, this->coeff_b,
                         this->coeff_c, this->coeff_d);
  }

  constexpr std::optional<float> calc_dx(const float x) const {
    return calc_first_derivative(x, this->x, this->coeff_b, this->coeff_c,
                                 this->coeff_d);
  }

  constexpr std::optional<float> calc_ddx(const float x) const {
    return calc_second_derivative(x, this->x, this->coeff_c, this->coeff_d);
  }

  constexpr std::optional<float> operator()(const float x) const {
    return this->calc_pos(x);
  }

private:
  const vector<x_len> x;
  const vector<x_len> y;
  const vector<x_len - 1> diffs;

  const matrix<x_len, 1> coeff_a;
  const matrix<x_len, 1> coeff_c;
  const matrix<x_len, 1> coeff_b;
  const matrix<x_len, 1> coeff_d;
};

template <std::size_t x_len>
constexpr vector<x_len>
calc_cumulative_arc_lengths(const vector<x_len> x,
                            const vector<x_len> y) noexcept {
  auto dx = calc_diffs(x);
  auto dy = calc_diffs(y);
  // np.hypot
  auto ds = cotila::elementwise(
      [](float xi, float yi) -> float {
        return cotila::sqrt(gcem::pow(xi, 2) + gcem::pow(yi, 2));
      },
      dx, dy);
  // np.cumsum
  for (std::size_t i = 1; i < ds.size; i++) {
    ds[i] = ds[i] + ds[i - 1];
  }
  const cotila::vector res = {0.0f};
  return cotila::concat(res, ds);
}

template <std::size_t x_len> class CubicSpline2d {
public:
  constexpr CubicSpline2d(vector<x_len> x, vector<x_len> y)
      : s(calc_cumulative_arc_lengths(x, y)), sx(CubicSpline1d(s, x)),
        sy(CubicSpline1d(s, y)) {}

  constexpr std::optional<std::pair<float, float>>
  calc_pos(const float s) const {
    auto x = this->sx.calc_pos(s);
    auto y = this->sy.calc_pos(s);
    if (!x.has_value() || !y.has_value()) {
      return std::nullopt;
    }

    return std::make_pair(x.value(), y.value());
  }

  constexpr std::optional<float> calc_curvature(const float s) const {
    auto dx = this->sx.calc_dx(s);
    auto ddx = this->sx.calc_ddx(s);
    auto dy = this->sy.calc_dx(s);
    auto ddy = this->sy.calc_ddx(s);

    if (!dx.has_value() || !ddx.has_value() || !dy.has_value() ||
        !ddy.has_value()) {
      return std::nullopt;
    }

    float k = (ddy.value() * dx.value() - ddx.value() * dy.value()) /
              gcem::pow((gcem::pow(dx.value(), 2) + gcem::pow(dy.value(), 2)),
                        (3 / 2));
    return k;
  }

  constexpr std::optional<float> calc_yaw(const float s) const {
    auto dx = this->sx.calc_dx(s);
    auto dy = this->sy.calc_dx(s);

    if (!dx.has_value() || !dy.has_value()) {
      return std::nullopt;
    }

    return gcem::atan2(dy.value(), dx.value());
  }

  template <std::size_t size>
  constexpr std::optional<std::pair<vector<size>, cotila::vector<float, size>>>
  calc_course() const {
    auto space = cotila::linspace<size>(0.f, this->end_limit());
    auto res_x = cotila::generate<size>([](int _) -> float { return 0.0; });
    auto res_y = cotila::generate<size>([](int _) -> float { return 0.0; });

    for (std::size_t i = 0; i < size; i++) {
      auto pos_opt = this->calc_pos(space[i]);
      if (!pos_opt.has_value()) {
        return std::nullopt;
      }

      auto pos = pos_opt.value();
      res_x[i] = pos.first;
      res_y[i] = pos.second;
    }

    return std::make_pair(res_x, res_y);
  }

  constexpr float end_limit() const { return this->s[this->s.size - 1]; }

  constexpr std::size_t course_size_from_step(const float step_size) const {
    return static_cast<std::size_t>(this->end_limit() / step_size);
  }

  constexpr std::optional<std::pair<float, float>>
  operator()(const float s) const {
    return this->calc_pos(s);
  }

private:
  const vector<x_len> s;
  const CubicSpline1d<x_len> sx;
  const CubicSpline1d<x_len> sy;
};

} // namespace voyage