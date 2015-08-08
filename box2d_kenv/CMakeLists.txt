cmake_minimum_required(VERSION 2.8.3)
project(box2d_kenv)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(Boost REQUIRED COMPONENTS system)
find_package(CGAL REQUIRED)
find_package(Eigen REQUIRED)
find_package(GEOS REQUIRED)
find_package(SFML REQUIRED COMPONENTS graphics system window)
find_package(YAMLCpp REQUIRED)

# The FindBox2D.cmake script does not set BOX2D_LIBRARIES.
find_package(Box2D REQUIRED)
set(BOX2D_LIBRARIES Box2D)

# GMP is a transitive dependency is CGAL that is not correctly specified in the
# current FindCGAL.cmake file.
find_package(GMP REQUIRED)
set(CGAL_INCLUDE_DIRS ${CGAL_INCLUDE_DIRS} ${GMP_INCLUDE_DIRS})
set(CGAL_LIBRARIES ${CGAL_LIBRARIES} ${GMP_LIBRARIES})

include(FindPkgConfig)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)

if ("${YamlCpp_VERSION}" VERSION_LESS 0.5.0)
  set(YAMLCPP_NEWAPI false)
  message(STATUS "Using the old-style yaml-cpp (< 0.5.0) API.")
else ()
  set(YAMLCPP_NEWAPI true)
  message(STATUS "Using the new-style yaml-cpp (>= 0.5.0) API.")
endif ()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS
    boost
    box2d-dev
    cgal
    eigen
    sfml-dev
)

###########
## Build ##
###########
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

# CGAL requires full IEEE 754 compliance.
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fp-model strict")
else ()
  message(WARNING "Your compiler was not recognized. We were unable to force"
                  " full IEEE 754 compliance for floating point operations"
                  " This may cause issues in CGAL.")
endif ()

# Generate a header that contains the YAMLCPP_NEWAPI flag.
configure_file(include/${PROJECT_NAME}/yaml_config.h.in
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/yaml_config.h
)

# Enable warnings.
set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

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
install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_demo
    ${PROJECT_NAME}_visualizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
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
