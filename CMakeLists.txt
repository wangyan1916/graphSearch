cmake_minimum_required(VERSION 3.10)

project(graphSearchTest VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(src)

add_executable(graphSearchTest main.cpp)

target_link_libraries(graphSearchTest PUBLIC GraphSearchBase)
target_include_directories(graphSearchTest PUBLIC "${PROJECT_SOURCE_DIR}/src")
