cmake_minimum_required(VERSION 2.8)

project(demo)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_executable(${PROJECT_NAME} "demo.cpp")

link_directories(${CMAKE_BINARY_DIR})
target_link_libraries (${PROJECT_NAME} LINK_PUBLIC ad5383_driver)
add_dependencies(${PROJECT_NAME} ad5383_driver)