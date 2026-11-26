# SPDX-License-Identifier: Apache-2.0
if(CONFIG_BIAS_DAC8568)
  zephyr_include_directories(.)
  zephyr_library()
  zephyr_library_sources(dac8568.c)
endif()
