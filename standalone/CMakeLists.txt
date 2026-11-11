cmake_minimum_required(VERSION 3.14...3.22 FATAL_ERROR)

project(NonPlanar3dMeshStandalone
	LANGUAGES CXX
)

include(../cmake/CPM.cmake)

CPMAddPackage(
	NAME NonPlanar3dMesh
	SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/..
)

file(GLOB_RECURSE headers
	CONFIGURE_DEPENDS
	"${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
)
file(GLOB_RECURSE sources
	CONFIGURE_DEPENDS
	"${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp"
)

add_executable(${PROJECT_NAME} ${sources})
set_target_properties(${PROJECT_NAME}
	PROPERTIES
	CXX_STANDARD 17
	OUTPUT_NAME "NonPlanar3dMeshstandalone"
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC NonPlanar3dMesh
)