cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "-g -Wall")

find_package(catkin REQUIRED COMPONENTS kenv gz_kenv quasistatic_pushing roscpp)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)

include (FindPkgConfig)
pkg_check_modules(GAZEBO gazebo)
pkg_check_modules(SDF sdformat)

catkin_package(
    INCLUDE_DIRS "include/" "proto_msg/"
    LIBRARIES "${PROJECT_NAME}" "${PROJECT_NAME}_msgs"
    CATKIN_DEPENDS gz_kenv quasistatic_pushing kenv
    DEPENDS eigen
)

include_directories(
    "include/${PROJECT_NAME}"
    "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
    ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDF_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
link_directories(
    ${Boost_LIBRARY_DIRS}
    ${GAZEBO_LIBRARY_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})

add_subdirectory(proto_msg)

add_library("${PROJECT_NAME}" SHARED
    src/Quasistatic_World_Plugin.cc
)
target_link_libraries("${PROJECT_NAME}"
    "${PROJECT_NAME}_msgs"
    ${BOOST_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
)
set_target_properties("${PROJECT_NAME}"
    PROPERTIES COMPILE_FLAGS -std=c++0x
)
