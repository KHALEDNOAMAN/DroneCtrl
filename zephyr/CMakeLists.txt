# DroneCtrl, Zephyr application build.
#
#   west build -b nucleo_f411re zephyr
#   west flash
#   west debug
#
# The include path reaches back into firmware/include, which is the whole point:
# one copy of the control law built twice, rather than a Zephyr fork that drifts
# from the ESP32 one within a month.
#
# The core is header-only, so there are no core sources to list here. That is
# also why the host tests can link it with no board, and why this port was
# possible at all.

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dronectrl_zephyr LANGUAGES C CXX)

target_sources(app PRIVATE
  src/main.c
  src/control_binding.cpp
)

target_include_directories(app PRIVATE
  src
  ${CMAKE_CURRENT_SOURCE_DIR}/../firmware/include
)

# Warnings as errors on the flight build only. The simulator and the host tests
# can be noisier; the thing that flies cannot.
target_compile_options(app PRIVATE
  -Wall -Wextra -Werror
  -Wdouble-promotion   # a float promoted to double on an M4F is a silent 10x cost
  -Wshadow
)
