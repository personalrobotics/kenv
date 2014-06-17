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

include_directories(include/kenv)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_DIRS})

find_package(PythonLibs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YamlCpp QUIET yaml-cpp)

#rosbuild_genmsg()
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

rosbuild_add_library(${PROJECT_NAME} src/OREnvironment.cpp)
target_link_libraries(${PROJECT_NAME} ${YamlCpp_LIBRARIES})

rosbuild_add_library(${PROJECT_NAME}_ext
    src/python/python.cpp
    src/python/python_OREnvironment.cpp
)
rosbuild_link_boost(${PROJECT_NAME}_ext python)
target_link_libraries(${PROJECT_NAME}_ext ${PROJECT_NAME} boost_numpy_eigen)
set_target_properties(${PROJECT_NAME}_ext PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/pythonsrc
    PREFIX ""
)
