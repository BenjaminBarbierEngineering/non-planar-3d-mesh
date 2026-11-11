#ifndef CURVATURE_BASED_SURFACE_ELEMENT_H
#define CURVATURE_BASED_SURFACE_ELEMENT_H

/**
 * @file curvature_based_surface_element.h
 * @brief Header file for the CurvatureBasedSurfaceElement structure.
 */

// Include necessary headers
#include <Eigen/Dense>
#include <vector>

namespace curvature_based_surface {
  /**
   * @brief Structure representing a curvature-based surface element.
   */
  struct CurvatureBasedSurfaceElement {
    Eigen::Vector3d position;  ///< The position of the 'initial' frenet-serret frame.
    Eigen::Vector3d
        normal;  ///< The normal vector of the 'initial' frenet-serret frame. This will help to
                 ///< position the osculating plane. To define the surface, there is an infinite
                 ///< possibilities of tangent vector combined to this normal vector.
    double curvature;             ///< The curvature of the 'initial' frenet-serret frame.
    double curvature_derivative;  ///< The derivative of the curvature of the 'initial'
                                  ///< frenet-serret frame. This curvature deriveative is constant.
  };
}  // namespace curvature_based_surface

#endif  // CURVATURE_BASED_SURFACE_ELEMENT_H