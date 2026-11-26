# SPDX-License-Identifier: Apache-2.0
if(CONFIG_BIAS_OLED_LITE)
  zephyr_library()
  zephyr_library_sources(oled_lite.c keys.c)
  zephyr_include_directories(.)
endif()
