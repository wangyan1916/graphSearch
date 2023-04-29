cmake_minimum_required(VERSION 3.10)

project(GraphSearchLib VERSION 0.1)

set(BASE_LIB GraphSearchBase)
add_library(${BASE_LIB} GraphSearch.cpp)