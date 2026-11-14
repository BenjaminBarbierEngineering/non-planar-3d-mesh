# Non-planar 3D mesh elements

This repository is an experimental C++ library for describing and sampling a local,
non-planar surface element. Conventional mesh elements are usually planar triangles
or other planar patches; this project explores representing a patch using a family
of curved trajectories instead.

## Surface model

The model is based on the Frenet-Serret frame: tangent, normal, and binormal
vectors, together with curvature. Torsion is assumed to be zero, so each
trajectory remains in its osculating plane. Curvature varies linearly with
arc length, with a constant curvature derivative. A third-order Taylor-Young
expansion of the Frenet-Serret equations approximates each trajectory near its
initial frame.

To form a surface patch, the initial tangent is rotated around the normal,
creating a family of osculating planes. Each trajectory is then evaluated at
sampled arc-length positions. The result is a flattened 3-by-N point matrix,
not a mesh with faces or connectivity.

## Header responsibilities

- `curvature_based_surface/curvature_based_surface_types.h` defines the data
  structures: `FrenetSerretFrameElement`, `CurvatureBasedSurfaceElement`, and
  `TaylorYoungFrenetSerretMatrices`.
- `curvature_based_surface/curvature_based_surface_utils.h` implements the
  geometric operations: building third-order expansion matrices, sampling a
  surface, choosing the initial tangent from a point, and estimating the frame
  closest to a point.
- `curvature_based_surface/curvature_based_surface_fitting.h` defines a Ceres
  residual for a point and `PointToCurvatureBasedSurfaceFittingProblem`, which
  assembles a least-squares fitting problem for a set of 3D observations.
- `utils/utils.h` provides `solve_polynomial`, which finds real roots using the
  eigenvalues of a polynomial companion matrix. It is used by the current
  closest-frame calculation.

The public geometric and fitting APIs are documented with Doxygen comments in
the headers.

## Using the library

The library is an INTERFACE (header-only) CMake target. Include the headers you
need and link your target to `NonPlanar3dMesh`. The project requires C++17,
Eigen 3.4 or newer, Ceres, and TBB. For example, to define and sample an
element:

```cpp
#include <Eigen/Dense>
#include "curvature_based_surface/curvature_based_surface_types.h"
#include "curvature_based_surface/curvature_based_surface_utils.h"

using namespace curvature_based_surface;

int main() {
  CurvatureBasedSurfaceElement<double> surface;
  surface.frenet_serret_frame.position = Eigen::Vector3d::Zero();
  surface.frenet_serret_frame.tangent = Eigen::Vector3d::UnitX();
  surface.frenet_serret_frame.normal = Eigen::Vector3d::UnitZ();
  surface.frenet_serret_frame.binormal = Eigen::Vector3d::UnitY();
  surface.frenet_serret_frame.curvature = 0.1;
  surface.curvature_derivative = 0.01;

  const auto points = compute_sample_curvature_based_surface(
      surface, 0.5, 10.0, M_PI / 12.0);
}
```

In an application CMake project, add this repository with `add_subdirectory`
or otherwise make the target available, then link it:

```cmake
add_subdirectory(path/to/non-planar-3d-mesh)
target_link_libraries(my_app PRIVATE NonPlanar3dMesh)
```

The fitting API is `PointToCurvatureBasedSurfaceFittingProblem`: construct it
with the number of input points, call `solve` with a 3-by-N point matrix, and
inspect the returned `CurvatureBasedSurfaceElement` and the problem's Ceres
summary. This fitting path and the geometric query routines are experimental;
see [Current limitations](#current-limitations) before relying on them.

## Building the standalone example

The standalone example is a separate CMake project. Configure and build it
from the repository root in an out-of-source build directory:

```sh
cmake -S standalone -B build/standalone
cmake --build build/standalone
```

Run the resulting executable:

```sh
./build/standalone/NonPlanar3dMeshstandalone
```

As with library consumers, CMake must be able to find Eigen 3.4 or newer, Ceres,
and TBB. The root project itself defines the library target; configuring the
`standalone` directory builds the demonstration executable.

## Current limitations

This is a research prototype, not a production-ready surface or meshing system.
In particular, the fitted model and query algorithms are not yet sufficiently
reliable for demanding geometry workloads. The library does not currently
provide a robust line/surface intersection API.

For closest-point queries, the current implementation derives the stationarity
condition for distance to the third-order curve approximation, yielding a
degree-six polynomial in arc length. That equation is deterministic, and its
real roots are sought through a companion-matrix eigenvalue solve. However,
the companion matrix can be ill-conditioned, making the numerical root solver
unreliable; consequently, the closest-frame result is not guaranteed to be
correct.

A more robust approach for closest-point and line-intersection queries would
formulate each query as a Ceres optimization problem. Running an optimizer for
each query is costly, though, so this approach is not efficient for workloads
that issue many queries, such as ray tracing or 3D-printing/slicing pipelines.
Fast and reliable query methods remain future work.
