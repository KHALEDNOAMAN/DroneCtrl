# GoogleTest suite for the portable flight code.
#
# This sits beside firmware/test rather than replacing it. The hand-rolled
# harness there needs nothing but a compiler, so the safety tests still run on
# a machine with no network. This suite trades that for what GoogleTest does
# well: one test body run over a whole table of inputs (TEST_P), the same test
# run for float and double (TYPED_TEST), and readable matchers.
#
#   cmake -S . -B build && cmake --build build && ctest --test-dir build
#
# GoogleTest is pinned to the commit of release v1.18.0, not to a branch, so a
# build today and a build next year compile the same test framework.

cmake_minimum_required(VERSION 3.16)
project(dronectrl_gtest CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        063de7e9578f82b369302001269680b4b1553359 # v1.18.0
)
# Windows: make GoogleTest use the same C runtime as the tests.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)

add_executable(flight_gtest
  mixer_test.cpp
  pid_test.cpp
  flight_state_test.cpp
  ekf_test.cpp
)
target_include_directories(flight_gtest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
if(NOT MSVC)
  target_compile_options(flight_gtest PRIVATE -Wall -Wextra -Wpedantic)
endif()
target_link_libraries(flight_gtest PRIVATE GTest::gtest_main GTest::gmock)

gtest_discover_tests(flight_gtest)
