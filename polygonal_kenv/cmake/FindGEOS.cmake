# - Try to find GEOS
# Once done this will define
#  GEOS_FOUND - System has GEOS
#  GEOS_INCLUDE_DIRS - The GEOS include directories
#  GEOS_LIBRARIES - The libraries needed to use GEOS

execute_process(COMMAND geos-config --includes
    OUTPUT_VARIABLE GEOS_INCLUDE_DIRS
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(COMMAND geos-config --libs
    OUTPUT_VARIABLE GEOS_LIBRARIES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package_handle_standard_args(GEOS
    DEFAULT_MSG
    GEOS_INCLUDE_DIRS
    GEOS_LIBRARIES
)
