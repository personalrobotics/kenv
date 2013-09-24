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
add_definitions(${EIGEN_DEFINITIONS})

add_definitions(-frounding-math)
include_directories(${PROJECT_SOURCE_DIR}/include/polygonal_kenv)

#rosbuild_genmsg()
#rosbuild_gensrv()

rosbuild_add_library(polygonal_kenv
    src/AffineTransformFilter.cpp
    src/PolygonalLink.cpp
    src/PolygonalEnvironment.cpp
    src/PolygonalViewer.cpp
    src/CSpaceObstacle.cpp
)
target_link_libraries(polygonal_kenv CGAL geos sfml-window sfml-graphics sfml-system yaml-cpp)
rosbuild_link_boost(polygonal_kenv system)

rosbuild_add_executable(planarize src/planarize.cpp)
rosbuild_link_boost(planarize program_options)
target_link_libraries(planarize polygonal_kenv yaml-cpp)

rosbuild_add_executable(test_polygonal src/test_polygonal.cpp)
rosbuild_link_boost(test_polygonal program_options thread)
target_link_libraries(test_polygonal polygonal_kenv geos yaml-cpp)
