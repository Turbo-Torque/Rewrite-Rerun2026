// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#pragma once

#include <cstddef>
#include <vector>
#include <stdexcept>
#include "Eigen/Core"
#include "Eigen/Dense"

namespace turbolib::util {

inline auto PerformRegression(const std::vector<double>& x, const std::vector<double>& y, unsigned int degree = 1) {
  if (x.empty() || y.empty()) {
    throw std::invalid_argument("Input vectors must not be empty");
  }

  if (x.size() != y.size()) {
    throw std::invalid_argument("x and y must have the same size");
  }

  if (degree < 1) {
    throw std::invalid_argument("Degree must be at least 1");
  }

  if (x.size() < static_cast<std::size_t>(degree + 1)) {
    throw std::invalid_argument("Number of data points must be at least degree + 1");
  }

  Eigen::MatrixXd A(x.size(), degree + 1);
  Eigen::VectorXd b(x.size());

  for (size_t i = 0; i < x.size(); ++i) {
    double x_value = x[i];
    for (unsigned int j = 0; j <= degree; ++j) {
      A(i, j) = std::pow(x_value, j);
    }
    b(i) = y[i];
  }

  Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);

  return [coeffs, degree](double x_val) {
    double result = coeffs[degree];
    for (int i = degree - 1; i >= 0; --i) {
      result = result * x_val + coeffs[i];
    }
    return result;
  };
}
}  // namespace turbolib::util
