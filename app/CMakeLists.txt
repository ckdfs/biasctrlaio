# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/..)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(biasctrl_h503)

target_sources(app PRIVATE
  src/main.c
  src/rt_pipeline.c
  src/cal_service.c
  src/ctrl_thread.c
  src/cli_service.c
)

execute_process(COMMAND git describe --always --dirty --exclude "*"
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  OUTPUT_VARIABLE firmware_revision OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY)
target_compile_definitions(app PRIVATE BIAS_FW_REV="${firmware_revision}")
