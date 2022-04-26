cmake_minimum_required(VERSION 2.8)

project(neutral)

add_executable(${PROJECT_NAME} "neutral.cpp")

link_directories(${CMAKE_BINARY_DIR})
target_link_libraries (${PROJECT_NAME} LINK_PUBLIC ad5383_driver)
