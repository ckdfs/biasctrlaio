set(control_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../../lib/control")
add_library(control_core STATIC
    "${control_dir}/ctrl_pid.c"
    "${control_dir}/ctrl_observer.c"
    "${control_dir}/ctrl_modulator.c")
target_include_directories(control_core PUBLIC "${control_dir}")
target_link_libraries(control_core PUBLIC dsp)
target_compile_options(control_core PRIVATE -Wall -Wextra -Werror)

foreach(suite IN ITEMS pid observer lock strategy)
    add_executable(control_${suite} test_${suite}.c)
    target_include_directories(control_${suite} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../dsp")
    target_link_libraries(control_${suite} PRIVATE control_core)
    target_compile_options(control_${suite} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME control_${suite} COMMAND control_${suite})
endforeach()
