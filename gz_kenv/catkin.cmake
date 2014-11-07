cmake_minimum_required(VERSION 2.8)

# NOTE: This is used by packages that depend on you. List of dependencies that
# dependencies might need. For Catkin and non-Catkin packages. INCLUDE_DIRS and
# LIBRARIES are exported from this package.
find_package(catkin REQUIRED COMPONENTS kenv)
find_package(Boost REQUIRED COMPONENTS system)
catkin_package(
    INCLUDE_DIRS "include/"
    LIBRARIES "${PROJECT_NAME}"
    CATKIN_DEPENDS kenv
    DEPENDS eigen
)
catkin_python_setup()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
link_directories(${GAZEBO_LIBRARY_DIRS})


find_package(Eigen REQUIRED)
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

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})



include_directories(${PROJECT_SOURCE_DIR}/include/gz_kenv)

add_library("${PROJECT_NAME}" SHARED src/gz_kenv.cpp)
target_link_libraries("${PROJECT_NAME}" yaml-cpp geos ${GAZEBO_LIBRARIES}  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties("${PROJECT_NAME}" PROPERTIES COMPILE_FLAGS -std=c++0x)

add_library("${PROJECT_NAME}_ext"
    src/python/python.cpp
    src/python/python_GazeboEnvironment.cpp
)
target_link_libraries("${PROJECT_NAME}_ext" ${catkin_LIBRARIES} ${Boost_LIBRARIES} kenv gz_kenv)

set_target_properties("${PROJECT_NAME}_ext" PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}"
    PREFIX ""
)                                             
