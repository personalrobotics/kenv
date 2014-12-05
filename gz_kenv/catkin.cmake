cmake_minimum_required(VERSION 2.8.3)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -g -O3")

# NOTE: This is used by packages that depend on you. List of dependencies that
# dependencies might need. For Catkin and non-Catkin packages. INCLUDE_DIRS and
# LIBRARIES are exported from this package.
find_package(catkin REQUIRED COMPONENTS kenv)
catkin_package(
    INCLUDE_DIRS "include/"
    LIBRARIES "${PROJECT_NAME}"
    CATKIN_DEPENDS kenv
    DEPENDS eigen
)
catkin_python_setup()


find_package(Eigen REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system python)


include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
link_directories(${GAZEBO_LIBRARY_DIRS})


include_directories(
 "include/gz_kenv"
 ${EIGEN_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}
 ${PYTHON_INCLUDE_DIRS}

)
add_definitions(${EIGEN_DEFINITIONS})



find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})



include_directories(${PROJECT_SOURCE_DIR}/include/gz_kenv)

add_library("${PROJECT_NAME}" SHARED src/gz_kenv.cpp)
target_link_libraries("${PROJECT_NAME}" yaml-cpp geos ${GAZEBO_LIBRARIES}  ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_library("${PROJECT_NAME}_ext"
    src/python/python.cpp
    src/python/python_GazeboEnvironment.cpp
)
target_link_libraries("${PROJECT_NAME}_ext" ${catkin_LIBRARIES} ${Boost_LIBRARIES} kenv gz_kenv)

set_target_properties("${PROJECT_NAME}_ext" PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}"
    PREFIX ""
)                             
