cmake_minimum_required(VERSION 3.20)
project(CellGen VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

set(THREEPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(THREEPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    threepp
    GIT_REPOSITORY https://github.com/markaren/threepp.git
    GIT_TAG 57aae69f08deba6e361dc8ec86c4ca6841fac43c
)
FetchContent_MakeAvailable(threepp)

find_package(assimp REQUIRED)

add_executable(CellGen
    src/main.cpp
    src/core/fence_catalog.cpp
    src/visualization/cell_renderer.cpp
    src/visualization/wall_system.cpp
    src/visualization/conveyor_system.cpp
    src/visualization/robot_system.cpp
)

target_include_directories(CellGen PRIVATE
    src
    ${threepp_SOURCE_DIR}/src/external/nlohmann
)
target_link_libraries(CellGen PRIVATE threepp::threepp assimp::assimp)
target_compile_options(CellGen PRIVATE -Wall -Wextra)
target_compile_definitions(CellGen PRIVATE
    CELLGEN_ASSETS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/assets"
)

# ---------------------------------------------------------------------------
# Tests (Google Test — no threepp dependency, pure logic only)
# ---------------------------------------------------------------------------

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(CellGenTests
    tests/test_fence_catalog.cpp
    tests/test_wall_layout.cpp
    src/core/fence_catalog.cpp
)

target_include_directories(CellGenTests PRIVATE
    src
    ${threepp_SOURCE_DIR}/src/external/nlohmann
)
target_link_libraries(CellGenTests PRIVATE GTest::gtest_main)
target_compile_options(CellGenTests PRIVATE -Wall -Wextra)

include(GoogleTest)
gtest_discover_tests(CellGenTests)
