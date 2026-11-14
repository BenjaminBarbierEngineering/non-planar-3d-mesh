#ifndef UTILS_H
#define UTILS_H

/**
 * @file utils.h
 * @brief Utility functions and definitions.
 */

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

namespace utils {
  /**
   * @brief Computes the real roots of a polynomial of degree 3 to 6.
   *
   * The coefficients are interpreted in descending powers, from the constant term to the highest
   * degree term, and the roots are recovered from the companion matrix eigenvalues.
   *
   * @tparam Derived Eigen expression type containing the polynomial coefficients.
   * @param coefficients Polynomial coefficients ordered from the constant term to the leading term.
   * @return Vector of real roots found for the polynomial.
   * @note The leading coefficient must be non-zero. If it is zero, the polynomial degree should be
   * reduced before calling this routine.
   * @note The implementation filters for real eigenvalues only; complex roots are discarded.
   */
  template <typename Derived> auto solve_polynomial(const Eigen::MatrixBase<Derived>& coefficients)
      -> Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> {
    using T = typename Derived::Scalar;
    Eigen::Matrix<T, Eigen::Dynamic, 1> changed_variable_coefficients{
        Eigen::Matrix<T, Eigen::Dynamic, 1>{coefficients.size(), 1}};
    T change_variable_factor = static_cast<T>(1e0);
    std::vector<int> power_vector(coefficients.size());
    std::iota(power_vector.begin(), power_vector.end(), 0);
    std::transform(std::execution::par_unseq, coefficients.cbegin(), coefficients.cend(),
                   power_vector.cbegin(), changed_variable_coefficients.begin(),
                   [&change_variable_factor](const T& coefficient_value, const int& power) {
                     return coefficient_value * std::pow(change_variable_factor, power);
                   });
    // Computation of polynom complex roots using companion matrix.
    std::vector<T> solutions;
    size_t nb_coeffs = coefficients.rows();
    // Creation of companion matrix
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> companion
        = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(nb_coeffs - 1, nb_coeffs - 1);
    companion.diagonal(-1) = 1. * Eigen::Matrix<T, Eigen::Dynamic, 1>::Ones(nb_coeffs - 2);
    companion.col(nb_coeffs - 2)
        = -coefficients.segment(0, nb_coeffs - 1) / coefficients[nb_coeffs - 1];

    // Eigen values of companion matrix are the complex roots of polynom.
    Eigen::EigenSolver<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> eigensolver(companion);
    auto eigen_val = eigensolver.eigenvalues();
    for (int i = 0; i < companion.rows(); ++i) {
      std::complex<T> sol{eigen_val.col(0)[i]};
      // Extraction of real solutions only (existing intersection)
      if (std::abs(sol) == abs(sol.real())) {
        solutions.push_back(sol.real());
      }
    }

    std::transform(solutions.begin(), solutions.end(), solutions.begin(),
                   std::bind(std::multiplies<T>(), std::placeholders::_1, change_variable_factor));
    Eigen::Matrix<T, Eigen::Dynamic, 1> solutions_mapped(
        static_cast<Eigen::Index>(solutions.size()));
    std::copy(solutions.begin(), solutions.end(), solutions_mapped.data());

    return solutions_mapped;
  }

}  // namespace utils

#endif  // UTILS_H