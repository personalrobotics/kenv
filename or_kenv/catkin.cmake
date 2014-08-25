cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS kenv)
catkin_package(
    INCLUDE_DIRS "include/"
    LIBRARIES "${PROJECT_NAME}"
    CATKIN_DEPENDS kenv
    DEPENDS boost eigen openrave
)
catkin_python_setup()

find_package(Boost REQUIRED COMPONENTS python system)
find_package(Eigen REQUIRED)
find_package(OpenRAVE REQUIRED)
find_package(PythonLibs)
pkg_check_modules(YamlCpp QUIET yaml-cpp)

#message(STATUS "Found OpenRAVE at ${OpenRAVE_LIBRARY_DIRS}")
message(STATUS "Found OpenRAVE at ${OpenRAVE_LIBRARY_DIRS}")

# TODO: The headers should be in "or_kenv", not "kenv".
include_directories(
    "${PROJECT_SOURCE_DIR}/include/kenv"
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
    ${OpenRAVE_INCLUDE_DIRS}
    ${YamlCpp_INCLUDE_DIRS}
)
link_directories(
    ${catkin_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
    ${EIGEN_LIBRARY_DIRS}
    ${PYTHON_LIBRARY_DIRS}
    ${OpenRAVE_LIBRARY_DIRS}
    ${YamlCpp_LIBRARY_DIRS}
)
add_definitions(
    ${EIGEN_DEFINITIONS}
    -frounding-math
)

# TODO: Don't hardcode openrave-core
add_library("${PROJECT_NAME}"
    src/OREnvironment.cpp
)
target_link_libraries("${PROJECT_NAME}"
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenRAVE_LIBRARIES}
    ${YamlCpp_LIBRARIES}
    "openrave${OpenRAVE_LIBRARY_SUFFIX}-core"
)

add_library("${PROJECT_NAME}_ext"
    src/python/python.cpp
    src/python/python_OREnvironment.cpp
)
target_link_libraries("${PROJECT_NAME}_ext"
    "${PROJECT_NAME}"
    boost_numpy_eigen
    ${Boost_LIBRARIES}
    ${OpenRAVE_LIBRARIES}
    "openrave${OpenRAVE_LIBRARY_SUFFIX}-core"
)
set_target_properties("${PROJECT_NAME}_ext" PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/pythonsrc"
    PREFIX ""
)
