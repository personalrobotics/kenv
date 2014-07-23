cmake_minimum_required(VERSION 2.8.3)

# NOTE: This is used by packages that depend on you. List of dependencies that
# dependencies might need. For Catkin and non-Catkin packages. INCLUDE_DIRS and
# LIBRARIES are exported from this package.
catkin_package(
    INCLUDE_DIRS "include/"
    LIBRARIES "${PROJECT_NAME}"
    DEPENDS eigen
)
catkin_python_setup()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

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
set_target_properties("${PROJECT_NAME}" PROPERTIES
    COMPILE_FLAGS -std=c++0x
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
    COMPILE_FLAGS -std=c++0x
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/pythonsrc"
    PREFIX ""
)
