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

  template <typename T> struct PointToCurvatureBasedSurfaceCostFunction {
    bool operator()(const T* const p0, const T* const normal_vector, const T* const tangent_vector,
                    const T* const curvature, const T* const curvature_derivative, const T* const s,
                    const T* const theta_angle, T* residuals) {
      return true;
    }
  };

}  // namespace curvature_based_surface

#endif  // CURVATURE_BASED_SURFACE_FITTING_H