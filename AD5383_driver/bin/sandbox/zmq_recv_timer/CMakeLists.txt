cmake_minimum_required(VERSION 3.18)

project(zmq_recv_timer_sandbox)

# place binaries and libraries according to GNU standards
include(GNUInstallDirs)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

find_package(cppzmq)
link_directories(${CMAKE_BINARY_DIR})

add_executable(
    ${PROJECT_NAME} 
    "main.cpp"
)
target_link_libraries(
    ${PROJECT_NAME} 
    LINK_PUBLIC cppzmq ad5383_driver
)
add_dependencies(
    ${PROJECT_NAME} 
    ad5383_driver
)


