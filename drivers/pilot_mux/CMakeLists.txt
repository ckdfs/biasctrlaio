# SPDX-License-Identifier: Apache-2.0
if(CONFIG_BIAS_PILOT_MUX)
  zephyr_library()
  zephyr_library_sources(pilot_mux.c pilot_mux_gpio.c)
  zephyr_include_directories(.)
endif()
