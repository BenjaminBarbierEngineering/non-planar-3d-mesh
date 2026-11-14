#ifndef CURVATURE_BASED_SURFACE_FITTING_H
#define CURVATURE_BASED_SURFACE_FITTING_H

/**
 * @file curvature_based_surface_fitting.h
 * @brief Header defining the curvature-based surface fitting problem and associated Ceres
 * residuals.
 *
 * @note This file uses the Ceres optimization library to solve for the local curve parameters of a
 * curvature-based surface from a set of 3D observations.
 */

#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <vector>

#include "curvature_based_surface/curvature_based_surface_types.h"
#include "curvature_based_surface/curvature_based_surface_utils.h"

namespace curvature_based_surface {

  /**
   * @brief Residual functor for fitting a curvature-based surface to a single 3D target point.
   *
   * The functor is used inside a Ceres optimization loop to evaluate how well the current frame
   * parameters reproduce the target point after a local Taylor-Young expansion of the Frenet-Serret
   * curve. It returns the world-space gap between the observed point and the predicted position on
   * the surface.
   */
  struct PointToCurvatureBasedSurfaceCostFunction {
    /**
     * @brief Construct a residual cost function for a target point.
     *
     * @param point Target point in the world frame that the optimized curve should reproduce.
     */
    PointToCurvatureBasedSurfaceCostFunction(const Eigen::Vector3d point)
        : point_to_fit_to(point) {}

    /**
     * @brief Evaluate the residual vector associated with the current frame parameters.
     *
     * Given the local origin, normal, curvature and arc-length parameter, the method reconstructs
     * the local Frenet-Serret frame, evaluates the Taylor-Young expansion, and stores the remaining
     * vector difference between the model prediction and the desired point.
     *
     * @tparam T Scalar type used by Ceres automatic differentiation, such as double or a Jet type.
     * @param p0 Current origin of the Frenet-Serret frame in world coordinates.
     * @param normal_vector Current normal vector of the frame.
     * @param curvature Current curvature magnitude of the local curve.
     * @param curvature_derivative Current derivative of the curvature with respect to arc length.
     * @param s Curvilinear abscissa at which the frame is evaluated.
     * @param residuals Residual vector containing the difference between the target point and the
     * predicted position on the curvature-based surface.
     *
     * @return Always true to indicate that the residual evaluation completed successfully.
     */
    template <typename T>
    bool operator()(const T* const p0, const T* const normal_vector, const T* const curvature,
                    const T* const curvature_derivative, const T* const s, T* residuals) const {
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
      Eigen::Matrix<T, 4, 1> derivative_coefficients;
      derivative_coefficients << static_cast<T>(1.), s[0], s[0] * s[0] / static_cast<T>(2.),
          s[0] * s[0] * s[0] / static_cast<T>(6.);

      auto mapped_residuals{Eigen::Map<Eigen::Matrix<T, 3, 1>>{residuals, 3, 1}};
      mapped_residuals
          = point - taylor_young_matrices.third_order_expansion_position * derivative_coefficients;
      return true;
    }

    Eigen::Vector3d point_to_fit_to;  ///< Target point in the world frame used to compute the
                                      ///< residual during the Ceres optimization.
  };

  /**
   * @brief Least-squares fitting problem for reconstructing a curvature-based surface from 3D data.
   *
   * This structure assembles a Ceres optimization where the unknowns are the origin of the initial
   * Frenet-Serret frame, the initial normal direction, the curvature, the curvature derivative, and
   * one curvilinear abscissa per observed point. Once configured, the optimization can solve for a
   * local surface element that best matches the supplied point cloud.
   */
  struct PointToCurvatureBasedSurfaceFittingProblem {
    /**
     * @brief Construct a fitting problem for a fixed number of points.
     *
     * @param number_of_points Number of 3D points to include in the optimization problem. This
     * value determines both the number of residual blocks and the size of the arc-length vector.
     */
    PointToCurvatureBasedSurfaceFittingProblem(const int& number_of_points)
        : points_to_fit(Eigen::Matrix3Xd{3, number_of_points}),
          s(Eigen::VectorXd::Zero(number_of_points)),
          curvature_based_cost_function(number_of_points) {
      // Create the auto differentiated cost functions and add the associated residual block
      for (long int i = 0; i < points_to_fit.cols(); i++) {
        curvature_based_cost_function[i]
            = new PointToCurvatureBasedSurfaceCostFunction(points_to_fit.col(i));
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<PointToCurvatureBasedSurfaceCostFunction, 3, 3, 3, 1, 1,
                                            1>(curvature_based_cost_function[i]),
            NULL, p0.data(), n0.data(), &curvature, &curvature_derivative, &s(i));
      }

      // Configure the optimization
      options.max_num_iterations = 50;
      options.num_threads = 4;
      options.linear_solver_type = ceres::SPARSE_SCHUR;

      // Change the ordering to be more efficient, dropping the less important parameters first
      auto* ordering = new ceres::ParameterBlockOrdering;
      for (long int i = 0; i < s.size(); i++) {
        ordering->AddElementToGroup(&s[i], 0);
      }
      ordering->AddElementToGroup(p0.data(), 1);
      ordering->AddElementToGroup(n0.data(), 1);
      ordering->AddElementToGroup(&curvature, 1);
      ordering->AddElementToGroup(&curvature_derivative, 1);
      options.linear_solver_ordering.reset(ordering);
    }

    /**
     * @brief Solve the fitting problem for a set of input points.
     *
     * The routine initializes the frame from the point cloud mean and principal variance direction,
     * runs the Ceres least-squares solve, and returns the optimized local surface element.
     *
     * @param input_points Matrix of size (3 x N) containing the 3D points to fit.
     *
     * @return Fitted curvature-based surface element obtained after optimization.
     */
    CurvatureBasedSurfaceElement<double> solve(Eigen::Matrix3Xd input_points) {
      // Initialize input points to fit
      points_to_fit = input_points;

      // Initialize parameters
      p0 = input_points.rowwise().mean();

      // Normal vector is initialized to the vector orientation that maximize the variance (PCA)
      Eigen::Matrix3Xd centered_points = input_points.colwise() - p0;
      // Non biased formula of covariance matrix
      Eigen::Matrix3d covariance_matrix = (centered_points * centered_points.transpose())
                                          / static_cast<double>(input_points.cols() - 1);

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance_matrix);
      if (eigen_solver.info() != Eigen::Success) {
        n0 = Eigen::Vector3d::UnitZ();
      } else {
        const auto eigen_vectors = eigen_solver.eigenvectors();
        n0 = eigen_vectors.col(2);
      }
      curvature = 0.;
      curvature_derivative = 0.;

      ceres::Solve(options, &problem, &summary);

      CurvatureBasedSurfaceElement<double> curvature_based_surface_element;
      curvature_based_surface_element.curvature_derivative = curvature_derivative;
      curvature_based_surface_element.frenet_serret_frame.curvature = curvature;
      curvature_based_surface_element.frenet_serret_frame.position = p0;
      curvature_based_surface_element.frenet_serret_frame.normal = n0;
      curvature_based_surface_element.frenet_serret_frame.tangent = n0.unitOrthogonal();
      curvature_based_surface_element.frenet_serret_frame.binormal
          = curvature_based_surface_element.frenet_serret_frame.tangent.cross(
              curvature_based_surface_element.frenet_serret_frame.normal);
      return curvature_based_surface_element;
    }

    // Input of the optimization problem
    Eigen::Matrix3Xd points_to_fit;  ///< Input points to fit, stored as a 3 x N matrix in the
                                     ///< world frame.

    // Parameters to optimize
    // For the initial FrenetSerretFrameElement
    Eigen::Vector3d
        p0;  ///< Initial position of the Frenet-Serret frame origin before optimization.
    Eigen::Vector3d n0;  ///< Initial normal vector used to initialize the local frame orientation.
    double curvature;    ///< Curvature parameter of the fitted curvature-based surface.
    double curvature_derivative;  ///< Derivative of the curvature, assumed constant along the
                                  ///< fitted curve.

    // For each point, we have to find the corresponding advancement
    Eigen::VectorXd s;  ///< Curvilinear abscissa associated with each point in the fitting problem.

    // Attributes for optimization
    std::vector<PointToCurvatureBasedSurfaceCostFunction*> curvature_based_cost_function;
    ceres::Problem problem;  ///< Ceres optimization problem holding the residual blocks.
    ceres::Solver::Options
        options;  ///< Solver configuration controlling the optimization strategy.
    ceres::Solver::Summary summary;  ///< Final solver summary returned after the optimization run.
  };
}  // namespace curvature_based_surface

#endif  // CURVATURE_BASED_SURFACE_FITTING_H
