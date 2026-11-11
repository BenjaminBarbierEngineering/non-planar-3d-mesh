# Non Planar 3D Mesh elements and grid
The purpose of this repository is to study the possibility to create a 3D mesh element with non planar characteristics. 
The classic meshing technics will use triangle planar elements and hybrid distribution. See classic meshing methods in https://en.wikipedia.org/wiki/Types_of_mesh.

Here we propose to study non planar elements.

## Curvature-based structural element

Based on the Frenet-Serret formulas, it is possible to describe a particle motion using a TNB basis (tangent, normal, binormal), curvature and torsion informations.

Using the Frenet-Serret formulas, we can determine the position of the particle in a neighborhood using this method.

Considering an osculating plane, in which the trajectory of a particle will vary, we can define a clothoid, a curve with a constant curvature derivative.

The osculating plane, if rotated, will define another curvature with the same parameters in 3D.