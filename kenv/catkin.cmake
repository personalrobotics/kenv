cmake_minimum_required(VERSION 2.8.3)

# NOTE: This is used by packages that depend on you. List of dependencies that
# dependencies might need. For Catkin and non-Catkin packages. INCLUDE_DIRS and
# LIBRARIES are exported from this package.
catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES ${PROJECT_NAME}
    #CATKIN_DEPENDS std_msgs owd_msgs geometry_msgs roscpp tf
    #DEPENDS libblas-dev liblapack-dev
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include/ include/kenv/ ${EIGEN3_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})

add_library(kenv
    src/Environment.cpp
    src/CollisionChecker.cpp
    src/ObjectPool.cpp
)
set_target_properties(kenv PROPERTIES COMPILE_FLAGS -std=c++0x)

add_library(kenv_ext
    src/python/python.cpp
    src/python/python_Environment.cpp
    src/python/python_CollisionChecker.cpp
    src/python/python_ObjectPool.cpp
)
target_link_libraries(kenv_ext kenv boost_numpy_eigen)
set_target_properties(kenv_ext PROPERTIES
    COMPILE_FLAGS -std=c++0x
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/pythonsrc
    PREFIX ""
)
