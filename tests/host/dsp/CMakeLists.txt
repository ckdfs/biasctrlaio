set(dsp_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/dsp")
add_library(dsp STATIC
    "${dsp_dir}/dsp_goertzel.c"
    "${dsp_dir}/dsp_pilot_gen.c"
    "${dsp_dir}/dsp_online_lsq.c")
target_include_directories(dsp PUBLIC "${dsp_dir}")
target_link_libraries(dsp PUBLIC m)
target_compile_options(dsp PRIVATE -Wall -Wextra -Werror)

foreach(suite IN ITEMS harmonics calibration)
    add_executable(test_dsp_${suite} test_${suite}.c)
    target_link_libraries(test_dsp_${suite} PRIVATE dsp)
    target_compile_options(test_dsp_${suite} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME dsp_${suite} COMMAND test_dsp_${suite})
endforeach()
