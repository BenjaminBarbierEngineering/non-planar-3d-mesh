#ifndef UTILS_H
#define UTILS_H

/**
 * @file utils.h
 * @brief Utility functions and definitions.
 */

#include <Eigen/Dense>

namespace utils {
  template <typename T> Eigen::Matrix<T, Eigen::Dynamic, 1> solve_polynomial(
      const Eigen::Matrix<T, Eigen::Dynamic, 1>& coefficients) {
    // This function should implement a polynomial solver for the given coefficients.
    // For simplicity, we can use Eigen's polynomial solver or any other method.
    // Here, we will just return an empty vector as a placeholder.
    return Eigen::Matrix<T, Eigen::Dynamic, 1>();
  }
}  // namespace utils

#endif  // UTILS_H