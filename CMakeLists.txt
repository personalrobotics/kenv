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

#rosbuild_genmsg()
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include/kenv)

rosbuild_add_library(kenv
    src/Environment.cpp
    src/CollisionChecker.cpp
    src/ObjectPool.cpp
)

rosbuild_add_library(kenv_ext
    src/python/python.cpp
    src/python/python_Environment.cpp
    src/python/python_CollisionChecker.cpp
    src/python/python_ObjectPool.cpp
)
rosbuild_link_boost(kenv_ext python)
# TODO: I should not need to explicitly link with glog here.
target_link_libraries(kenv_ext kenv boost_numpy_eigen glog)
set_target_properties(kenv_ext PROPERTIES PREFIX "")
