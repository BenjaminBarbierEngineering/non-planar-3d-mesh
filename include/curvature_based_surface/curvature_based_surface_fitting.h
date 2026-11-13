#ifndef CURVATURE_BASED_SURFACE_FITTING_H
#define CURVATURE_BASED_SURFACE_FITTING_H

/**
 * @file curvature_based_surface_fitting.h
 * @brief Header file for curvature-based surface fitting structures and methods.
 *
 * @note This file use Ceres dependency for optimization. Ceres is a C++ library for modeling and
 * solving large, complicated optimization problems.
 */

#include <ceres/ceres.h>

#include <Eigen/Dense>

#include "curvature_based_surface/curvature_based_surface_types.h"
#include "curvature_based_surface/curvature_based_surface_utils.h"

namespace curvature_based_surface {

  struct PointToCurvatureBasedSurfaceCostFunction {
    PointToCurvatureBasedSurfaceCostFunction(const Eigen::Vector3d point)
        : point_to_fit_to(point) {}

    template <typename T>
    bool operator()(const T* const p0, const T* const normal_vector, const T* const curvature,
                    const T* const curvature_derivative, const T* const s, T* residuals) {
      // point_to_fit_to is in double, so we need to cast it to T.
      Eigen::Matrix<T, 3, 1> point = point_to_fit_to.cast<T>();

      // First, we need to compute the first Frenet-Serret frame element from the parameters.
      FrenetSerretFrameElement<T> first_frame_element;
      first_frame_element.position << p0[0], p0[1], p0[2];
      first_frame_element.normal << normal_vector[0], normal_vector[1], normal_vector[2];
      // The tangent vector is computed using the utility function available :
      first_frame_element.tangent = compute_first_tangent_vector(first_frame_element.position,
                                                                 first_frame_element.normal, point);
      first_frame_element.binormal = first_frame_element.tangent.cross(first_frame_element.normal);
      first_frame_element.curvature = curvature[0];

      auto taylor_young_matrices = compute_taylor_young_frenet_serret_matrices(
          first_frame_element, curvature_derivative[0]);
      Eigen::Matrix<T, 4, 1> derivative_coefficients << 1, s, s * s / 2, s * s * s / 6;

      auto mapped_residuals{Eigen::Map<Eigen::Matrix<T, 3, 1>>{residuals, 3, 1}};
      mapped_residuals
          = point - taylor_young_matrices.third_order_expansion_position * derivative_coefficients;
      return true;
    }

    Eigen::Vector3d point_to_fit_to;  ///< One point on which the curvature-based surface is fitted.
                                      ///< This point is in the world frame.
  };

}  // namespace curvature_based_surface

#endif  // CURVATURE_BASED_SURFACE_FITTING_H