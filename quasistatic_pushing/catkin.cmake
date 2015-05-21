cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS kenv)
catkin_package(
    INCLUDE_DIRS "include/"
    LIBRARIES "${PROJECT_NAME}"
    CATKIN_DEPENDS kenv
    DEPENDS eigen
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

include(FindPkgConfig)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)
if (${YamlCpp_VERSION} VERSION_LESS 0.5.0)
    message(STATUS "Using the old-style yaml-cpp (< 0.5.0) API.")
else ()
    add_definitions(-DYAMLCPP_NEWAPI)
    message(STATUS "Using the new-style yaml-cpp (>= 0.5.0) API.")
endif ()

include_directories(
    "include/quasistatic_pushing"
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}
)

add_library("${PROJECT_NAME}"
    src/QuasistaticPushingModel.cpp
)
target_link_libraries("${PROJECT_NAME}"
    ${catkin_LIBRARIES}
)
