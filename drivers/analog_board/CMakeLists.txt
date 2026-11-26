# SPDX-License-Identifier: Apache-2.0
if(CONFIG_BIAS_ANALOG_BOARD_20261110_MZM)
  zephyr_library()
  zephyr_library_sources(mzm_20261110.c)
  zephyr_include_directories(.)
endif()
