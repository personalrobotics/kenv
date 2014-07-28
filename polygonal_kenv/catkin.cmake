cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS kenv)
catkin_package(
    INCLUDE_DIRS "include/"
    LIBRARIES "${PROJECT_NAME}"
    CATKIN_DEPENDS kenv
    DEPENDS cgal eigen
)
catkin_python_setup()

include(FindPkgConfig)

find_package(Boost REQUIRED COMPONENTS python system)
find_package(CGAL REQUIRED)
find_package(Eigen REQUIRED)
find_package(PythonLibs)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)

include_directories(
    "${PROJECT_SOURCE_DIR}/include/polygonal_kenv"
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CGAL_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${YamlCpp_INCLUDE_DIRS}
)
add_definitions(
    ${EIGEN_DEFINITIONS}
    -frounding-math
)

add_library("${PROJECT_NAME}"
    src/AffineTransformFilter.cpp
    src/PolygonalLink.cpp
    src/PolygonalEnvironment.cpp
    src/CSpaceObstacle.cpp
)
target_link_libraries("${PROJECT_NAME}"
    # TODO: Shouldn't CGAL_LIBRARIES contain this?
    CGAL
    # TODO: Figure out how to find GEOS in a portable way.
    geos
    ${catkin_LIBRARIES}
    ${CGAL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${YamlCpp_LIBRARIES}
)

add_library("${PROJECT_NAME}_ext" SHARED
    src/python/python.cpp
    src/python/python_PolygonalEnvironment.cpp
)
target_link_libraries("${PROJECT_NAME}_ext"
    "${PROJECT_NAME}"
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES}
)
set_target_properties("${PROJECT_NAME}_ext" PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/pythonsrc"
    PREFIX ""
)
