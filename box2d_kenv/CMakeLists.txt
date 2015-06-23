cmake_minimum_required(VERSION 2.8.3)
project(box2d_kenv)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(FindPkgConfig)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
  message(STATUS "Defaulting to CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
endif ()

find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Box2D REQUIRED)
find_package(CGAL REQUIRED)
find_package(GEOS REQUIRED)
find_package(Eigen REQUIRED)
find_package(SFML REQUIRED COMPONENTS graphics system window)
find_package(YAMLCpp REQUIRED)

# GMP is a transitive dependency is CGAL that is not correctly specified in the
# current FindCGAL.cmake file.
find_package(GMP REQUIRED)

# This is necessary for the yaml_utils.h header.
# TODO: Move this into kenv.
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)

if (${YamlCpp_VERSION} VERSION_LESS 0.5.0)
  message(STATUS "Using the old-style yaml-cpp (< 0.5.0) API.")
else ()
  add_definitions(-DYAMLCPP_NEWAPI)
  message(STATUS "Using the new-style yaml-cpp (>= 0.5.0) API.")
endif ()

# TODO: This should be handled by find_package(Box2D)
set(BOX2D_LIBRARIES Box2D)

include_directories(SYSTEM
  include
  include/${PROJECT_NAME}
  ${BOOST_INCLUDE_DIRS}
  ${BOX2D_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${GEOS_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${SFML_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  ${BOOST_LIBRARY_DIRS}
  ${BOX2D_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${GEOS_LIBRARY_DIRS}
  ${CGAL_LIBRARY_DIRS}
  ${SFML_LIBRARY_DIRS}
  ${YAMLCPP_LIBRARY_DIR}
  ${catkin_LIBRARY_DIRS}
)

# TODO: This should be handled by find_package(CGAL)
add_definitions(-frounding-math)

set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS boost eigen box2d-dev cgal sfml-dev
)

###########
## Build ##
###########
add_library(${PROJECT_NAME}
  src/Box2DBody.cpp
  src/Box2DFactory.cpp
  src/Box2DJoint.cpp
  src/Box2DLink.cpp
  src/Box2DSensor.cpp
  src/Box2DSensorMonitor.cpp
  src/Box2DWorld.cpp
  src/geometry_utils.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${BOOST_LIBRARIES}
  ${BOX2D_LIBRARIES}
  ${CGAL_LIBRARIES}
  ${GMP_LIBRARIES}
  ${GEOS_LIBRARIES}
  ${YAMLCPP_LIBRARY}
)

add_library(${PROJECT_NAME}_visualizer
  src/Box2DVisualizer.cpp
)
target_link_libraries(${PROJECT_NAME}_visualizer
  ${BOOST_LIBRARIES}
  ${BOX2D_LIBRARIES}
  ${SFML_LIBRARIES}
)

add_executable(${PROJECT_NAME}_demo
  src/test_box2d_kenv.cpp
)
target_link_libraries(${PROJECT_NAME}_demo
  ${PROJECT_NAME}
  ${PROJECT_NAME}_visualizer
  ${BOOST_LIBRARIES}
  ${BOX2D_LIBRARIES}
  ${SFML_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_visualizer ${PROJECT_NAME}_demo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
 )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_box2d_kenv.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
