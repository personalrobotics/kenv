cmake_minimum_required(VERSION 2.8.3)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -g -O3")

# NOTE: This is used by packages that depend on you. List of dependencies that
# dependencies might need. For Catkin and non-Catkin packages. INCLUDE_DIRS and
# LIBRARIES are exported from this package.
catkin_package(
    INCLUDE_DIRS "include/"
    LIBRARIES "${PROJECT_NAME}"
    DEPENDS eigen
)
catkin_python_setup()

include(FindPkgConfig)
pkg_check_modules(YamlCpp REQUIRED yaml-cpp)

if (${YamlCpp_VERSION} VERSION_LESS 0.5.0)
    message(STATUS "Using the old-style yaml-cpp (< 0.5.0) API.")
else ()
    add_definitions(-DYAMLCPP_NEWAPI)
    message(STATUS "Using the new-style yaml-cpp (>= 0.5.0) API.")
endif ()

find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system python)

include(DetectCXX11Flags)

include_directories(
    "include/"
    "include/kenv/"
    ${EIGEN3_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIRS}
)

add_library("${PROJECT_NAME}"
    src/Environment.cpp
    src/CollisionChecker.cpp
    src/Jacobian.cpp
    src/ObjectPool.cpp
)

add_library("${PROJECT_NAME}_ext"
    src/python/python.cpp
    src/python/python_Environment.cpp
    src/python/python_CollisionChecker.cpp
    src/python/python_ObjectPool.cpp
)
target_link_libraries("${PROJECT_NAME}_ext"
    "${PROJECT_NAME}"
    boost_numpy_eigen
)
set_target_properties("${PROJECT_NAME}_ext" PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}"
    PREFIX ""
)