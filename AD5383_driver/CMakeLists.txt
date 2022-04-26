cmake_minimum_required(VERSION 2.8)

project(ad5383_driver)

set(CMAKE_CXX_FLAGS "-O2 -std=c++11")

message("${CMAKE_SYSTEM_NAME}")

set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(SOURCES_DIR ${CMAKE_SOURCE_DIR}/src)

# A. Include directory
include_directories(${INCLUDE_DIR})

# C. Developped libraries
add_library(ad5383 ${SOURCES_DIR}/ad5383.cpp)

set(AD5383_LIB ${CMAKE_BINARY_DIR}/ad5383_driver.a)
add_custom_target(combined
        COMMAND ar -x $<TARGET_FILE:ad5383>
        COMMAND ar -qcs ${AD5383_LIB} *.o
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        DEPENDS ad5383
)

add_library(ad5383_driver STATIC IMPORTED GLOBAL)
add_dependencies(ad5383_driver combined)
set_target_properties(ad5383_driver
        PROPERTIES
        IMPORTED_LOCATION ${AD5383_LIB}
)

# D. C executable files directories
add_subdirectory("bin/ad5383_driver")
add_subdirectory("bin/neutral")
