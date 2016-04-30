cmake_minimum_required(VERSION 2.8.3)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -g -O3")

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system python)

# NOTE: This is used by packages that depend on you. List of dependencies that
# dependencies might need. For Catkin and non-Catkin packages. INCLUDE_DIRS and
# LIBRARIES are exported from this package.
catkin_package(
    INCLUDE_DIRS "include/"
    LIBRARIES "${PROJECT_NAME}"
    DEPENDS eigen
)
catkin_python_setup()

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
    src/ObjectPool.cpp
)
