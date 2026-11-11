cmake_minimum_required(VERSION 3.14...3.22 FATAL_ERROR)

project(NonPlanar3dMesh
	VERSION 0.0.1
	LANGUAGES CXX
)

if(PROJECT_BINARY_DIR STREQUAL PROJECT_SOURCE_DIR)
	message(FATAL_ERROR
		"In-source builds are not allowed. Please create a separate build directory."
	)
endif()

include(cmake/CPM.cmake)
find_package(Eigen3 3.4.0 REQUIRED)

file(GLOB_RECURSE headers
	CONFIGURE_DEPENDS
	"${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
)
file(GLOB_RECURSE sources
	CONFIGURE_DEPENDS
	"${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp"
)

add_library(${PROJECT_NAME} ${headers} ${sources})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)

# Add Warnings
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	target_compile_options(${PROJECT_NAME}
		PRIVATE
		$<$<COMPILE_LANGUAGE:CXX>:-Wall>
		$<$<COMPILE_LANGUAGE:CXX>:-Wextra>
		$<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
		$<$<COMPILE_LANGUAGE:CXX>:-Werror>
	)
endif()

target_include_directories(${PROJECT_NAME}
	PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}-${PROJECT_VERSION}>
)

target_link_libraries(${PROJECT_NAME}
	PRIVATE Eigen3::Eigen
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_DOCUMENTATION "Build the Doxygen API documentation" OFF)

if(BUILD_DOCUMENTATION)
	add_subdirectory(documentation)
endif()