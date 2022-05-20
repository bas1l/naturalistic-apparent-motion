cmake_minimum_required(VERSION 3.18)

project(ad5383_driver)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_FLAGS "-O2 -std=c++11 -pthread")

message("${CMAKE_SYSTEM_NAME}")


find_package(Boost 1.79.0 COMPONENTS iostreams system)
#find cppzmq wrapper, installed by make of cppzmq
find_package(cppzmq)


# A. Include directory
set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
set(SOURCES_DIR ${CMAKE_SOURCE_DIR}/src)
include_directories(${INCLUDE_DIR} ${Boost_INCLUDE_DIR})

# C. Developped libraries
add_library(ad5383 ${SOURCES_DIR}/ad5383.cpp)
target_link_libraries(ad5383 ${Boost_LIBRARIES})

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
add_subdirectory("bin/actuators_driver_dir")
add_subdirectory("bin/neutral_dir")
add_subdirectory("bin/sandbox/zerozmq")
add_subdirectory("bin/sandbox/zerozmq_receiver")
add_subdirectory("bin/sandbox/zmq_recv_timer")
add_subdirectory("bin/listener_for_python")
