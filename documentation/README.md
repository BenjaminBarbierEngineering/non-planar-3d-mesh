# API documentation

This directory contains the CMake target for generating the project's API
documentation from the public headers in `include/`.

## Requirements

- CMake 3.14 or newer
- Doxygen
- The project build dependencies (including Eigen3)

## Generate the documentation

From the repository root, configure a build directory with documentation
enabled, then build the documentation target:

```sh
cmake -S . -B build/documentation -DBUILD_DOCUMENTATION=ON
cmake --build build/documentation --target NonPlanar3dMesh_documentation
```

Open `build/documentation/html/index.html` in a browser to view the generated
HTML documentation. The generated files are in the build directory and are not
added to the source tree. The user can use the following command to launch the html result :

```sh
python3 -m http.server 8000 --directory build/documentation/documentation/html
```

## Documenting functions

Add Doxygen-style comments to declarations in the public headers. Use `@brief`
for a short description and tags such as `@param` and `@return` for function
inputs and results. For example:

```cpp
/**
 * @brief Prints a message to the standard output.
 * @param message The message to print.
 */
void printMessage(const std::string& message);
```

When documentation is enabled, CMake requires Doxygen to be installed. Leave
`BUILD_DOCUMENTATION` off for regular builds that do not need to generate the
documentation.
