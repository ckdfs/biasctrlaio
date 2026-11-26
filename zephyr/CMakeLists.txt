# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(../include ../lib/dsp ../lib/control)
foreach(driver pilot_mux dac8568 ads131m02 eeprom_board oled_lite analog_board)
  set(driver_dir ${CMAKE_CURRENT_LIST_DIR}/../drivers/${driver})
  if(IS_DIRECTORY ${driver_dir})
    add_subdirectory(${driver_dir} ${CMAKE_CURRENT_BINARY_DIR}/${driver})
  endif()
endforeach()

zephyr_library_named(bias_dsp_control)
zephyr_library_sources(
  ../lib/dsp/dsp_goertzel.c
  ../lib/dsp/dsp_online_lsq.c
  ../lib/dsp/dsp_pilot_gen.c
  ../lib/control/ctrl_pid.c
  ../lib/control/ctrl_observer.c
  ../lib/control/ctrl_modulator.c
  ../lib/control/ctrl_mod_mzm.c
  ../lib/control/ctrl_bias_engine.c
)
