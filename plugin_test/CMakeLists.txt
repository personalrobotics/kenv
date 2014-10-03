cmake_minimum_required(VERSION 2.8.3)
project(plugin_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_quasistatic_plugin
  gz_kenv
  kenv
  roscpp
)


find_package(Boost REQUIRED COMPONENTS system)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
find_package(Eigen REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
link_directories(${GAZEBO_LIBRARY_DIRS})

set (CMAKE_CXX_FLAGS "-g -Wall")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
include_directories(
 "include/"
 ${EIGEN_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})
add_library(plugin_test src/Pusher_Plugin.cc)
target_link_libraries(plugin_test ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
set_target_properties(plugin_test PROPERTIES COMPILE_FLAGS -std=c++0x)