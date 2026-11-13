#ifndef CURVATURE_BASED_SURFACE_TYPES_H
#define CURVATURE_BASED_SURFACE_TYPES_H

/**
 * @file curvature_based_surface_types.h
 * @brief Header file for the curvature-based surface types (element, frame, and expansion
 * matrices).
 */

// Include necessary headers
#include <Eigen/Dense>
#include <vector>

namespace curvature_based_surface {

  /**
   * @brief Structure representing a Frenet-Serret frame element (t, n, b, and curvature, as torsion
   * is 0).
   *
   * @tparam T The data type for the position, tangent, normal, binormal, and curvature. Could be
   * float, double or Jet.
   */
  template <typename T> struct FrenetSerretFrameElement {
    Eigen::Matrix<T, 3, 1> position;  ///< The position of the frenet-serret frame.
    Eigen::Matrix<T, 3, 1> tangent;   ///< The tangent vector of the frenet-serret frame.
    Eigen::Matrix<T, 3, 1> normal;    ///< The normal vector of the frenet-serret frame.
    Eigen::Matrix<T, 3, 1> binormal;  ///< The binormal vector of the frenet-serret frame.
    T curvature;                      ///< The curvature of the frenet-serret frame.
  };

  /**
   * @brief Structure representing a curvature-based surface element.
   *
   * @tparam T The data type for the position, normal, curvature, and curvature derivative. Could be
   * float, double or Jet.
   */
  template <typename T> struct CurvatureBasedSurfaceElement {
    FrenetSerretFrameElement<T>
        frenet_serret_frame;  ///< The frenet-serret initial frame of the curvature-based surface
                              ///< element, corresponding to a 0 radian rotation around the normal
                              ///< vector.
    T curvature_derivative;   ///< The derivative of the curvature of the 'initial' frenet-serret
                              ///< frame. This curvature deriveative is constant.
  };

  /**
   * @brief Structure representing the Taylor-Young expansion matrices to the third order for a
   * Frenet-Serret frame, using the Frenet-Serret formulas. See
   * https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas for more details.
   *
   * @note This is a helper. For example, to extract the real position, one should apply a matrix
   * product with the vector (1, s, s^2/2, s^3/6)^T, where s is the distance along the curve.
   */
  template <typename T> struct TaylorYoungFrenetSerretMatrices {
    Eigen::Matrix<T, 3, 4>
        third_order_expansion_position;  ///< The third order expansion matrix of the position
                                         ///< vector of the frenet-serret frame, composed with (p0,
                                         ///< dp0/ds, d²p0/ds², d³p0/ds³).
    Eigen::Matrix<T, 3, 4>
        third_order_expansion_tangent;  ///< The third order expansion matrix of the tangent vector
                                        ///< of the frenet-serret frame, composed with (t0, dt0/ds,
                                        ///< d²t0/ds², d³t0/ds³).
    Eigen::Matrix<T, 3, 4>
        third_order_expansion_normal;  ///< The third order expansion matrix of the normal vector of
                                       ///< the frenet-serret frame, composed with (n0, dn0/ds,
                                       ///< d²n0/ds², d³n0/ds³).
  };
}  // namespace curvature_based_surface

#endif  // CURVATURE_BASED_SURFACE_TYPES_H