# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20)
project(pilot_mux_tests LANGUAGES C)
enable_testing()
set(root "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
add_executable(test_pilot_mux test_pilot_mux.c "${root}/drivers/pilot_mux/pilot_mux.c")
target_include_directories(test_pilot_mux PRIVATE "${root}/drivers/pilot_mux" "${root}/include")
set_target_properties(test_pilot_mux PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
add_test(NAME pilot_mux COMMAND test_pilot_mux)
