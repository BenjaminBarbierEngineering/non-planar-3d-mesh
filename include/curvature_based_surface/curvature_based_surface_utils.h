#ifndef CURVATURE_BASED_SURFACE_UTILS_H
#define CURVATURE_BASED_SURFACE_UTILS_H

/**
 * @file curvature_based_surface_utils.h
 * @brief Header file for utility functions related to curvature-based surfaces.
 */

// Include necessary headers
#include <Eigen/Dense>
#include <algorithm>
#include <execution>
#include <iostream>
#include <vector>

#include "curvature_based_surface/curvature_based_surface_types.h"
#include "utils/utils.h"

namespace curvature_based_surface {

  /**
   * @brief Expands a Frenet-Serret frame up to third order in arc-length.
   *
   * Uses the Frenet-Serret relations to construct the Taylor-Young expansion matrices for the
   * position, tangent, and normal vectors around a given frame element.
   *
   * @tparam T The scalar type used by the frame data (for example float, double, or Jet).
   * @param frame_element_initial Initial Frenet-Serret frame used as the expansion center.
   * @param curvature_derivative First derivative of the curvature with respect to arc length.
   *
   * @return Third-order expansion matrices for the position, tangent, and normal fields.
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
    Eigen::Matrix<T, 3, 1> t3 = k0 * n2 + static_cast<T>(2.) * curvature_derivative * n1;
    Eigen::Matrix<T, 3, 1> n3 = -k0 * t2 - static_cast<T>(2.) * curvature_derivative * t1;

    TaylorYoungFrenetSerretMatrices<T> matrices;

    // Fill the expansion matrices
    matrices.third_order_expansion_position << p0, p1, p2, p3;
    matrices.third_order_expansion_tangent << t0, t1, t2, t3;
    matrices.third_order_expansion_normal << n0, n1, n2, n3;

    return matrices;
  }

  /**
   * @brief Samples a curvature-based surface by sweeping a Frenet-Serret frame around the normal.
   *
   * For each angular sample, the initial tangent is rotated in the osculating plane and the surface
   * is expanded locally using the third-order Taylor-Young development of the underlying curve.
   *
   * @tparam T The scalar type used by the sampled geometry (for example float, double, or Jet).
   * @param curvature_based_surface Surface definition containing the base frame, curvature, and
   * derivative information.
   * @param advancement_sampling_step Arc-length increment used along the curve for sampling.
   * @param advancement_max Maximum arc-length reached when traversing the generated surface.
   * @param angle_sampling_step Angular step used when rotating the tangent around the local normal.
   *
   * @return Flattened point cloud of the sampled surface, stored as a 3 x N matrix.
   */
  template <typename T> Eigen::Matrix<T, 3, Eigen::Dynamic> compute_sample_curvature_based_surface(
      const CurvatureBasedSurfaceElement<T>& curvature_based_surface, T advancement_sampling_step,
      T advancement_max, T angle_sampling_step) {
    Eigen::Matrix<T, Eigen::Dynamic, 1> angle_vector
        = Eigen::Matrix<T, Eigen::Dynamic, 1>::LinSpaced(
            static_cast<int>(round(2. * M_PI / angle_sampling_step)), static_cast<T>(0.),
            static_cast<T>(2. * M_PI));
    Eigen::Matrix<T, Eigen::Dynamic, 1> advancement_vector
        = Eigen::Matrix<T, Eigen::Dynamic, 1>::LinSpaced(
            static_cast<int>(round(advancement_max / advancement_sampling_step)),
            static_cast<T>(0.), advancement_max);

    std::vector<Eigen::Matrix<T, 3, Eigen::Dynamic>> vector_of_points(
        angle_vector.size(), Eigen::Matrix<T, 3, Eigen::Dynamic>{3, advancement_vector.size()});

    std::transform(std::execution::par_unseq, angle_vector.cbegin(), angle_vector.cend(),
                   vector_of_points.begin(),
                   [&curvature_based_surface, &advancement_vector](const T& angle) {
                     FrenetSerretFrameElement<T> first_frame_element
                         = curvature_based_surface.frenet_serret_frame;
                     // Rotate the tangent around the normal vector
                     first_frame_element.tangent
                         = cos(angle) * curvature_based_surface.frenet_serret_frame.tangent
                           + sin(angle) * curvature_based_surface.frenet_serret_frame.binormal;
                     first_frame_element.binormal
                         = first_frame_element.tangent.cross(first_frame_element.normal);
                     auto taylor_young_matrices = compute_taylor_young_frenet_serret_matrices(
                         first_frame_element, curvature_based_surface.curvature_derivative);

                     Eigen::Matrix<T, 3, Eigen::Dynamic> points_on_curve{
                         Eigen::Matrix<T, 3, Eigen::Dynamic>{3, advancement_vector.size()}};
                     for (Eigen::Index i = 0; i < advancement_vector.size(); ++i) {
                       const T advancement = advancement_vector[i];
                       Eigen::Matrix<T, 4, 1> advancement_coefficient_expansion;
                       advancement_coefficient_expansion << static_cast<T>(1.), advancement,
                           advancement * advancement / static_cast<T>(2.),
                           advancement * advancement * advancement / static_cast<T>(6.);
                       points_on_curve.col(i) = taylor_young_matrices.third_order_expansion_position
                                                * advancement_coefficient_expansion;
                     }
                     return points_on_curve;
                   });
    Eigen::Matrix<T, 3, Eigen::Dynamic> output_points{
        Eigen::Matrix<T, 3, Eigen::Dynamic>{3, angle_vector.size() * advancement_vector.size()}};
    for (long int i = 0; i < vector_of_points.size(); i++) {
      output_points.block(0, i * advancement_vector.size(), 3, advancement_vector.size())
          = vector_of_points[i];
    }
    return output_points;
  }

  /**
   * @brief Computes the tangent that orients the osculating plane of a point.
   *
   * The returned tangent is the normalized projection of the vector from the initial position to
   * the target point onto the plane orthogonal to the given normal.
   *
   * @tparam T The scalar type used by the vectors (for example float, double, or Jet).
   * @param initial_position Position used to define the osculating plane.
   * @param initial_normal Normal vector constraining the plane to a single tangent direction.
   * @param point Point fixing the remaining degree of freedom, thereby defining the tangent.
   *
   * @return Tangent vector aligned with the osculating plane and normalized to unit length.
   */
  template <typename T> Eigen::Matrix<T, 3, 1> compute_first_tangent_vector(
      const Eigen::Matrix<T, 3, 1>& initial_position, const Eigen::Matrix<T, 3, 1>& initial_normal,
      const Eigen::Matrix<T, 3, 1>& point) {
    Eigen::Matrix<T, 3, 1> vector_element_to_initial_point = point - initial_position;

    Eigen::Matrix<T, 3, 1> resulting_tangent
        = vector_element_to_initial_point
          - vector_element_to_initial_point.dot(initial_normal) * initial_normal;
    resulting_tangent.normalize();
    return resulting_tangent;
  }

  /**
   * @brief Finds the Frenet-Serret frame element closest to a given point in space.
   *
   * The frame is first re-oriented so that the point lies in its osculating plane, then the optimal
   * arc-length parameter is recovered by solving the polynomial equation associated with the local
   * Taylor-Young expansion of the curve.
   *
   * @tparam T The scalar type used by the frame and point data (for example float, double, or Jet).
   * @param surface_element Curvature-based surface used to define the local frame and curvature
   * derivative.
   * @param point Query point for which the closest frame is sought.
   *
   * @return Frenet-Serret frame element whose position is closest to the input point.
   */
  template <typename T> FrenetSerretFrameElement<T> compute_closer_frame_to_point(
      const CurvatureBasedSurfaceElement<T>& surface_element, const Eigen::Matrix<T, 3, 1>& point) {
    // First, retrieve the corresponding first tangent, orienting the osculating plane of the first
    // frame element to expand. The point should be in this plane.
    // Then, create the first frame element from which to expand.
    FrenetSerretFrameElement<T> first_frame_element = surface_element.frenet_serret_frame;
    first_frame_element.tangent
        = compute_first_tangent_vector(surface_element.frenet_serret_frame.position,
                                       surface_element.frenet_serret_frame.normal, point);
    first_frame_element.binormal = first_frame_element.tangent.cross(first_frame_element.normal);

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
    polynomial_coefficients(0)
        = point_tangent_product(0) - static_cast<T>(2.) * dr_dt_product(0, 0);
    polynomial_coefficients(1) = point_tangent_product(1)
                                 - static_cast<T>(2.) * (dr_dt_product(0, 1) + dr_dt_product(1, 0));
    polynomial_coefficients(2)
        = point_tangent_product(2) / static_cast<T>(2.)
          - static_cast<T>(2.)
                * ((dr_dt_product(0, 2) + dr_dt_product(2, 0)) / static_cast<T>(2.)
                   + dr_dt_product(1, 1));
    polynomial_coefficients(3)
        = point_tangent_product(3) / static_cast<T>(6.)
          - static_cast<T>(2.)
                * ((dr_dt_product(0, 3) + dr_dt_product(3, 0)) / static_cast<T>(6.)
                   + (dr_dt_product(1, 2) + dr_dt_product(2, 1) / static_cast<T>(2.)));
    polynomial_coefficients(4) = -static_cast<T>(2.)
                                 * ((dr_dt_product(1, 3) + dr_dt_product(3, 1)) / static_cast<T>(6.)
                                    + dr_dt_product(2, 2) / static_cast<T>(4.));
    polynomial_coefficients(5)
        = -static_cast<T>(2.) * ((dr_dt_product(2, 3) + dr_dt_product(3, 2)) / static_cast<T>(12.));
    polynomial_coefficients(6) = -static_cast<T>(2.) * (dr_dt_product(3, 3) / static_cast<T>(36.));

    /// TODO: Implement a polynomial solver for the given coefficients. For now, it is a placeholder
    /// (empty vector).
    auto all_closest_s_computed = utils::solve_polynomial(polynomial_coefficients);
    std::cout << "all_closest_s_computed:" << std::endl;
    std::cout << all_closest_s_computed << std::endl;

    // Now, we need to find the closest s among all the computed ones.
    T closest_s = static_cast<T>(0.);
    T closest_distance = std::numeric_limits<T>::max();
    for (int i = 0; i < all_closest_s_computed.size(); ++i) {
      T s = all_closest_s_computed(i);
      Eigen::Matrix<T, 4, 1> s_vector;
      s_vector << 1, s, s * s / static_cast<T>(2.), s * s * s / static_cast<T>(6.);
      Eigen::Matrix<T, 3, 1> position_s
          = taylor_young_matrices.third_order_expansion_position * s_vector;
      T distance = (point - position_s).norm();
      if (distance < closest_distance) {
        closest_distance = distance;
        closest_s = s;
      }
    }

    // Now, we can compute the closest frame element to the point using the closest s.
    FrenetSerretFrameElement<T> closest_frame_element;
    Eigen::Matrix<T, 4, 1> closest_s_vector;
    closest_s_vector << 1, closest_s, closest_s * closest_s / static_cast<T>(2.),
        closest_s * closest_s * closest_s / static_cast<T>(6.);
    closest_frame_element.position
        = taylor_young_matrices.third_order_expansion_position * closest_s_vector;
    closest_frame_element.tangent
        = taylor_young_matrices.third_order_expansion_tangent * closest_s_vector;
    closest_frame_element.normal
        = taylor_young_matrices.third_order_expansion_normal * closest_s_vector;
    closest_frame_element.binormal
        = first_frame_element.binormal;  // The binormal is constant along the curve.
    closest_frame_element.curvature
        = first_frame_element.curvature + surface_element.curvature_derivative * closest_s;

    return closest_frame_element;
  }
}  // namespace curvature_based_surface

#endif  // CURVATURE_BASED_SURFACE_UTILS_H