# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_LIST_DIR}/../..)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(biasctrl_ztest_p4)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/src
  ${CMAKE_CURRENT_LIST_DIR}/../../app/src
  ${CMAKE_CURRENT_LIST_DIR}/../../drivers/ads131m02
  ${CMAKE_CURRENT_LIST_DIR}/../../drivers/dac8568
  ${CMAKE_CURRENT_LIST_DIR}/../../drivers/analog_board
)

target_sources(app PRIVATE
  src/main.c
  src/mocks.c
  ../../app/src/rt_pipeline.c
  ../../app/src/cal_service.c
  ../../app/src/ctrl_thread.c
  ../../app/src/cli_service.c
)
