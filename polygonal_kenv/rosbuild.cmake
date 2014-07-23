cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS} -frounding-math)

find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include/polygonal_kenv)

#rosbuild_genmsg()
#rosbuild_gensrv()

rosbuild_add_library(polygonal_kenv
    src/AffineTransformFilter.cpp
    src/PolygonalLink.cpp
    src/PolygonalEnvironment.cpp
    src/CSpaceObstacle.cpp
)
target_link_libraries(polygonal_kenv CGAL geos yaml-cpp)
rosbuild_link_boost(polygonal_kenv system)

rosbuild_add_library(polygonal_kenv_ext
    src/python/python.cpp
    src/python/python_PolygonalEnvironment.cpp
)
rosbuild_link_boost(polygonal_kenv_ext python)
target_link_libraries(polygonal_kenv_ext kenv polygonal_kenv)
set_target_properties(polygonal_kenv_ext PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/pythonsrc
    PREFIX ""
)
