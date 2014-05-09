cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})


find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include/gz_kenv)


rosbuild_add_library(gz_kenv SHARED src/gz_kenv.cpp)
target_link_libraries(gz_kenv yaml-cpp geos ${GAZEBO_LIBRARIES})
set_target_properties(gz_kenv PROPERTIES COMPILE_FLAGS -std=c++0x)
rosbuild_link_boost(gz_kenv system)

rosbuild_add_library(gz_kenv_ext
    src/python/python.cpp
    src/python/python_GazeboEnvironment.cpp
)
rosbuild_link_boost(gz_kenv_ext python)
target_link_libraries(gz_kenv_ext kenv gz_kenv)
set_target_properties(gz_kenv_ext PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/pythonsrc
    PREFIX ""
)

