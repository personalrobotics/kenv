cmake_minimum_required(VERSION 2.8.3)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -g -O3")

include(DetectCXX11Flags)
include(FindPkgConfig)

find_package(catkin REQUIRED COMPONENTS cmake_modules kenv)
find_package(Boost REQUIRED COMPONENTS python system)
find_package(CGAL REQUIRED)
find_package(GEOS REQUIRED)
find_package(Eigen REQUIRED)
find_package(PythonLibs REQUIRED)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)

if (${YamlCpp_VERSION} VERSION_LESS 0.5.0)
    message(STATUS "Using the old-style yaml-cpp (< 0.5.0) API.")
else ()
    add_definitions(-DYAMLCPP_NEWAPI)
    message(STATUS "Using the new-style yaml-cpp (>= 0.5.0) API.")
endif ()

catkin_package(
    INCLUDE_DIRS "include/"
    LIBRARIES "${PROJECT_NAME}"
    CATKIN_DEPENDS
    DEPENDS
        cgal
        eigen
)
catkin_python_setup()

include_directories(
    "${PROJECT_SOURCE_DIR}/include/polygonal_kenv"
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CGAL_INCLUDE_DIR}
    ${GEOS_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${YamlCpp_INCLUDE_DIRS}
)
add_definitions(
    ${EIGEN_DEFINITIONS}
)

add_library("${PROJECT_NAME}"
    src/AffineTransformFilter.cpp
    src/PolygonalLink.cpp
    src/PolygonalEnvironment.cpp
    src/CSpaceObstacle.cpp
)
target_link_libraries("${PROJECT_NAME}"
    # TODO: Shouldn't CGAL_LIBRARIES contain this?
    ${GEOS_LIBRARIES}
    ${catkin_LIBRARIES}
    ${CGAL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${YamlCpp_LIBRARIES}
)
