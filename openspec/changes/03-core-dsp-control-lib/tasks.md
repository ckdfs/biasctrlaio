# Tasks: Phase 3 — Portable C11 DSP & Control Algorithm Library

- [x] p3_lib_dsp Port and extend `lib/dsp/` (`dsp_goertzel`, dual-pilot `dsp_pilot_gen`, `O(1)`-RAM `dsp_online_lsq`) with host CTest unit tests
  owned: lib/dsp/, tests/host/dsp/, tests/host/CMakeLists.txt
  depends:
  verify: cmake -S tests/host -B build/host-tests -G Ninja && cmake --build build/host-tests && ctest --test-dir build/host-tests --output-on-failure -R dsp

- [x] p3_lib_control_core Port and decouple `lib/control/` (`ctrl_pid`, `ctrl_observer`, `ctrl_modulator.h` vtable) preserving all invariants with host unit tests
  owned: lib/control/, tests/host/control/
  depends: p3_lib_dsp
  verify: cmake --build build/host-tests && ctest --test-dir build/host-tests --output-on-failure -R control

- [x] p3_mod_mzm_and_sim Implement `ctrl_mod_mzm`, `ctrl_bias_engine`, and host MZM optical closed-loop simulator covering QUAD/MAX/MIN/CUSTOM targets
  owned: lib/control/, tests/host/sim/
  depends: p3_lib_control_core
  verify: cmake --build build/host-tests && ctest --test-dir build/host-tests --output-on-failure
