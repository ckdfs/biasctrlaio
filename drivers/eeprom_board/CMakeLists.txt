# SPDX-License-Identifier: Apache-2.0
if(CONFIG_BIAS_EEPROM_BOARD OR CONFIG_BIAS_OLED_LITE)
  zephyr_library()
  zephyr_library_sources(shared_i2c.c)
  zephyr_library_sources_ifdef(CONFIG_BIAS_EEPROM_BOARD eeprom_board.c)
  zephyr_include_directories(.)
endif()
