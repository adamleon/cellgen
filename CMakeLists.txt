cmake_minimum_required(VERSION 3.20)
project(CellGen VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(CellGen
    src/main.cpp
)

target_compile_options(CellGen PRIVATE -Wall -Wextra -Wpedantic)
