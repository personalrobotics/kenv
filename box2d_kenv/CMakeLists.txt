cmake_minimum_required(VERSION 2.8.3)
project(box2d_kenv)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Box2D REQUIRED)
find_package(GEOS REQUIRED)
find_package(Eigen REQUIRED)
find_package(SFML REQUIRED COMPONENTS graphics system window)

# CGAL requires the -frounding-math option.
# TODO: What should we do for CLang? It doesn't support -frounding-math.
find_package(CGAL REQUIRED)
set(CMAKE_C_FLAGS -frounding-math)
set(CMAKE_CXX_FLAGS -frounding-math)

# The YamlCpp API dramatically changed in version 0.5.
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)

if (${YamlCpp_VERSION} VERSION_LESS 0.5.0)
  message(STATUS "Using the old-style yaml-cpp API. (${YamlCpp_VERSION} < 0.5.0)")
else ()
  add_definitions(-DYAMLCPP_NEWAPI)
  message(STATUS "Using the new-style yaml-cpp API (${YamlCpp_VERSION} >= 0.5.0).")
endif ()

# GMP is a transitive dependency is CGAL that is not correctly specified in the
# current FindCGAL.cmake file.
find_package(GMP REQUIRED)

message(STATUS "Box2D_LIBRARIES = ${Box2D_LIBRARIES}")
message(STATUS "BOX2D_LIBRARIES = ${BOX2D_LIBRARIES}")
message(STATUS "Box2D_LIBRARY = ${Box2D_LIBRARY}")
message(STATUS "BOX2D_LIBRARY = ${BOX2D_LIBRARY}")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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

include_directories(
  include/${PROJECT_NAME}
  ${BOOST_INCLUDE_DIRS}
  ${BOX2D_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${GEOS_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${SFML_INCLUDE_DIRS}
  ${YamlCpp_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/Box2DBody.cpp
  src/Box2DLink.cpp
  src/Box2DJoint.cpp
  src/Box2DWorld.cpp
  src/Box2DFactory.cpp
  src/geometry_utils.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${BOOST_LIBRARIES}
  ${BOX2D_LIBRARY}
  ${CGAL_LIBRARIES}
  ${GMP_LIBRARIES}
  ${GEOS_LIBRARIES}
  ${YamlCpp_LIBRARIES}
)

add_library(${PROJECT_NAME}_visualizer
  src/Box2DVisualizer.cpp
)
target_link_libraries(${PROJECT_NAME}_visualizer
  ${BOOST_LIBRARIES}
  ${BOX2D_LIBRARY}
  ${SFML_LIBRARIES}
)

add_executable(${PROJECT_NAME}_demo
  src/test_box2d_kenv.cpp
)
target_link_libraries(${PROJECT_NAME}_demo
  ${PROJECT_NAME}
  ${PROJECT_NAME}_visualizer
  ${BOOST_LIBRARIES}
  ${BOX2D_LIBRARY}
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
