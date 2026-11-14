
// clang-format off
#include "curvature_based_surface/curvature_based_surface_types.h"
#include "curvature_based_surface/curvature_based_surface_utils.h"
#include "curvature_based_surface/curvature_based_surface_fitting.h"

#include <iostream>
// clang-format on

using namespace curvature_based_surface;

int main() {
  // First create the initial CurvatureBasedSurfaceElement
  CurvatureBasedSurfaceElement<double> curvature_based_surface_element;
  curvature_based_surface_element.curvature_derivative = 0.1;
  curvature_based_surface_element.frenet_serret_frame.curvature = 0.1;
  curvature_based_surface_element.frenet_serret_frame.position = Eigen::Vector3d::Zero();
  curvature_based_surface_element.frenet_serret_frame.tangent = Eigen::Vector3d::UnitX();
  curvature_based_surface_element.frenet_serret_frame.normal = Eigen::Vector3d::UnitZ();
  curvature_based_surface_element.frenet_serret_frame.binormal = Eigen::Vector3d::UnitY();

  // Constant definitiion
  static constexpr double advancement_sampling_step{0.5};
  static constexpr double advancement_max{100.};
  static constexpr double angle_sampling_step{M_PI / 6.};

  Eigen::Matrix3Xd points_on_surface = compute_sample_curvature_based_surface(
      curvature_based_surface_element, advancement_sampling_step, advancement_max,
      angle_sampling_step);

  PointToCurvatureBasedSurfaceFittingProblem curvature_surface_problem(points_on_surface.size());
  auto computed_curvature_surface = curvature_surface_problem.solve(points_on_surface);

  std::cout << curvature_surface_problem.summary.FullReport() << std::endl;

  std::cout << "Curvature surface fitting finished" << std::endl;
  std::cout << "Point : " << computed_curvature_surface.frenet_serret_frame.position.transpose()
            << std::endl;
  std::cout << "Normal : " << computed_curvature_surface.frenet_serret_frame.normal.transpose()
            << std::endl;
  std::cout << "Curvature : " << computed_curvature_surface.frenet_serret_frame.curvature
            << std::endl;
  std::cout << "Curvature derivative : " << computed_curvature_surface.curvature_derivative
            << std::endl;

  // Sample a point at advancement 20. and angle 0.
  auto taylor_matrices = compute_taylor_young_frenet_serret_matrices(
      curvature_based_surface_element.frenet_serret_frame,
      curvature_based_surface_element.curvature_derivative);
  Eigen::Vector4d adv_values;
  double advancement_sample = 20.;
  adv_values << 1., advancement_sample, advancement_sample * advancement_sample / 2.,
      advancement_sample * advancement_sample * advancement_sample / 6.;
  Eigen::Vector3d resulting_position = taylor_matrices.third_order_expansion_position * adv_values;
  Eigen::Vector3d resulting_normal = taylor_matrices.third_order_expansion_normal * adv_values;
  std::cout << resulting_position << std::endl;

  Eigen::Vector3d position_to_test = resulting_position + 1. * resulting_normal;

  // Compute the closest advancement, should be equal to advancement sample :
  auto frenet_serret_closer
      = compute_closer_frame_to_point(curvature_based_surface_element, position_to_test);
  std::cout << "Closest computed :" << std::endl;
  std::cout << frenet_serret_closer.position << std::endl;

  return 0;
}