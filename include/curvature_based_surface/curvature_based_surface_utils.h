#ifndef CURVATURE_BASED_SURFACE_UTILS_H
#define CURVATURE_BASED_SURFACE_UTILS_H

/**
 * @file curvature_based_surface_utils.h
 * @brief Header file for utility functions related to curvature-based surfaces.
 */
* /
// Include necessary headers
#include <Eigen/Dense>
#include <vector>

#include "curvature_based_surface/curvature_based_surface_types.h"

    namespace curvature_based_surface {

  /**
   * @brief Computes the Taylor-Young expansion matrices to the third order for a Frenet-Serret
   * frame, using the Frenet-Serret formulas.
   *
   * @tparam T The data type for the position, tangent, normal, binormal, and curvature. Could be
   * float, double or Jet.
   * @param frame_element_initial The initial Frenet-Serret frame element, from which to expand.
   * @param curvature_derivative The derivative of the curvature.
   *
   * @return The Taylor-Young expansion matrices for the Frenet-Serret frame.
   */
  template <typename T>
  TaylorYoungFrenetSerretMatrices<T> compute_taylor_young_frenet_serret_matrices(
      const FrenetSerretFrameElement<T>& frame_element_initial, T curvature_derivative) {
    // Initial tangent, normal, and curvature from the Frenet-Serret frame element
    const auto& p0 = frame_element_initial.position;
    const auto& t0 = frame_element_initial.tangent;
    const auto& n0 = frame_element_initial.normal;
    const auto& k0 = frame_element_initial.curvature;

    // First order derivatives
    Eigen::Matrix<T, 3, 1> p1 = t0;
    Eigen::Matrix<T, 3, 1> t1 = k0 * n0;
    Eigen::Matrix<T, 3, 1> n1 = -k0 * t0;

    // Second order derivatives
    Eigen::Matrix<T, 3, 1> p2 = t1;
    Eigen::Matrix<T, 3, 1> t2 = k0 * n1 + curvature_derivative * n0;
    Eigen::Matrix<T, 3, 1> n2 = -k0 * t1 - curvature_derivative * t0;

    // Third order derivatives
    Eigen::Matrix<T, 3, 1> p3 = t2;
    Eigen::Matrix<T, 3, 1> t3 = k0 * n2 + 2 * curvature_derivative * n1;
    Eigen::Matrix<T, 3, 1> n3 = -k0 * t2 - 2 * curvature_derivative * t1;

    TaylorYoungFrenetSerretMatrices<T> matrices;

    // Fill the expansion matrices
    matrices.third_order_expansion_position << p0, p1, p2, p3;
    matrices.third_order_expansion_tangent << t0, t1, t2, t3;
    matrices.third_order_expansion_normal << n0, n1, n2, n3;

    return matrices;
  }

  FrenetSerretFrameElement<T> compute_closer_frame_to_point(
      CurvatureBasedSurfaceElement<T> surface_element, Eigen::Matrix<T, 3, 1> point) {
    // First, retrieve the corresponding first tangent, orienting the osculating plane of the first
    // frame element to expand. The point should be in this plane.
    // Then, create the first frame element from which to expand.
    Eigen::Matrix<T, 3, 1> vector_element_to_point
        = point - surface_element.frenet_serret_frame.position;
    FrenetSerretFrameElement<T> first_frame_element;
    first_frame_element.position = surface_element.frenet_serret_frame.position;
    first_frame_element.tangent
        = vector_element_to_point
          - vector_element_to_point.dot(surface_element.frenet_serret_frame.normal)
                * surface_element.frenet_serret_frame.normal;
    first_frame_element.tangent.normalize();
    first_frame_element.normal = surface_element.frenet_serret_frame.normal;
    first_frame_element.binormal = first_frame_element.tangent.cross(first_frame_element.normal);
    first_frame_element.curvature = surface_element.frenet_serret_frame.curvature;

    // clang-format off
    // Given the closest frame element to the point at curvilinear abscissa s, expanding the above
    // frame element to the third order, we have the following equation that are verified :
    //
    // (point - position(s))^t * tangent(s) = position(s)^t * tangent(s)
    //
    // knowing that : 
    //
    // tangent(s) = third_order_expansion_tangent * (1, s, s^2/2, s^3/6)^t
    // position(s) = third_order_expansion_position * (1, s, s^2/2, s^3/6)^t 
    //
    // We have :
    //
    // point^t * third_order_expansion_tangent * (1, s, s^2/2, s^3/6)^t - 2 * (1, s, s^2/2, s^3/6) * third_order_expansion_position^t * third_order_expansion_tangent * (1, s, s^2/2, s^3/6)^t = 0
    //
    // And this is a polynomial of degree 6 in s, which we can solve to find the closest frame
    // element to the point.
    // clang-format on
    auto taylor_young_matrices = compute_taylor_young_frenet_serret_matrices(
        first_frame_element, surface_element.curvature_derivative);

    Eigen::Matrix<T, 4, 4> dr_dt_product
        = taylor_young_matrices.third_order_expansion_position.transpose()
          * taylor_young_matrices.third_order_expansion_tangent;
    Eigen::Matrix<T, 1, 4> point_tangent_product
        = point.transpose() * taylor_young_matrices.third_order_expansion_tangent;

    // Now retrieve the coefficients of the polynomial of degree 6 in s, and solve it.
    // In the dr_dt_product matrix, the coefficients are ordered by anti diagonals.
    Eigen::Matrix<T, 7, 1> polynomial_coefficients;
    polynomial_coefficients(0) = point_tangent_product(0) - 2 * dr_dt_product(0, 0);
    polynomial_coefficients(1)
        = point_tangent_product(1) - 2 * (dr_dt_product(0, 1) + dr_dt_product(1, 0));
    polynomial_coefficients(2)
        = point_tangent_product(2) / 2
          - 2 * ((dr_dt_product(0, 2) + dr_dt_product(2, 0)) / 2 + dr_dt_product(1, 1));
    polynomial_coefficients(3) = point_tangent_product(3) / 6
                                 - 2
                                       * ((dr_dt_product(0, 3) + dr_dt_product(3, 0)) / 6
                                          + (dr_dt_product(1, 2) + dr_dt_product(2, 1) / 2));
    polynomial_coefficients(4)
        = -2 * ((dr_dt_product(1, 3) + dr_dt_product(3, 1)) / 6 + dr_dt_product(2, 2) / 4);
    polynomial_coefficients(5) = -2 * ((dr_dt_product(2, 3) + dr_dt_product(3, 2)) / 12);
    polynomial_coefficients(6) = -2 * (dr_dt_product(3, 3) / 36);
  }

}  // namespace curvature_based_surface

#endif  // CURVATURE_BASED_SURFACE_UTILS_H