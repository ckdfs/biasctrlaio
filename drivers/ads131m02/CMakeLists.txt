# SPDX-License-Identifier: Apache-2.0
if(CONFIG_BIAS_ADS131M02)
  zephyr_include_directories(.)
  zephyr_library()
  zephyr_library_sources(ads131m02.c)
  zephyr_library_sources_ifdef(CONFIG_SOC_SERIES_STM32H5X ads131m02_dma.c)
endif()
