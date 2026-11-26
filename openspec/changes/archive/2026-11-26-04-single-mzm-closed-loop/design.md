# Design: Phase 4 — Single-MZM 64 kSPS Real-Time Closed-Loop Control

## Thread & Interrupt Architecture
1. **`DRDY#` Fast Path (Highest Priority, `15.625 us` period)**:
   - Triggered on `PA11` falling edge.
   - Reads `ADS131M02` 12-byte frame via `SPI2` DMA, feeds `CH0` into Goertzel `f0`/`2*f0` and accumulates `CH1` DC monitor sum, pulls next `DITHER1` sample from `dsp_pilot_gen`, and writes `DAC8568` `CH6` (`VOUTG`) via `SPI1` DMA.
   - Every `1280` samples (`20 ms`), posts a fixed-size block result struct to `k_msgq` without blocking; increments `drop_count` and triggers `FAULT` if `k_msgq` overflows or any DMA/CRC timeout occurs.
   - Zero calls to `i2c_*`, `printk`, `k_sleep`, or mutexes on the `64 kSPS` path.
2. **Control Thread (`ctrl_thread`, Priority 5, `2.5 KiB` stack)**:
   - Consumes `10` blocks (`5 Hz`), runs `ctrl_bias_engine`, updates `VA` DC bias on `DAC8568` `CH0`, or steps the two-pass `cal bias` state machine using `dsp_online_lsq`.
   - On any fault, immediately pulls all 8 `TS5A23159` switches LOW and resets bias outputs safely.
3. **CLI & Telemetry Thread (`cli_service`, Priority 8, `2.0 KiB` stack)**:
   - Interrupt/ring-buffer driven `USART1` command parser (`status`, `cal bias`, `start`, `stop`, `target quad|min|max|custom <deg>`, `save`, `load`, `scan vpi`).
   - Persists/restores calibration records to `U21` (`P24C02A` via `eeprom_board`).

## RT Pipeline Contract (p4_rt_pipeline)
- **Driver & DSP Integration**:
  - Uses `bias_ads131m02_start()` (`<ads131m02.h>`) for the `64 kSPS` `DRDY#` + `SPI2` DMA callback and `analog_board_ops_t::set_pilot_code(0, code)` (`<bias/analog_board.h>`) for ISR-safe `DITHER1` `SPI1` DMA output.
  - Uses `lib/dsp`: two `goertzel_state_t` bins (`f0 = 1000 Hz`, `2*f0 = 2000 Hz`, `fs = 64000 Hz`, `N = 1280`), one `dc_accum_t` (`N = 1280`) for `CH1`, and `pilot_gen_t` (`f0 = 1000 Hz`, `fs = 64000 Hz`).
  - Converts 24-bit signed `ADS131M02` counts to volts (using `ADS131M02` `+/-1.2 V` reference scale `1.2f / 8388608.0f`, configurable/explicit constant) and `DITHER1` volts to 16-bit `DAC8568` codes centered around mid-scale (`32768` for `0 V` bipolar or `0..2.5 V` internal-ref scale, clearly documented in `rt_pipeline.h`).
- **Block Handoff (`50 Hz`)**:
  - Every `1280` samples, posts a fixed-size `rt_pipeline_block_t` (containing `harmonic_data_t`, `uint32_t seq`, `uint32_t timestamp_cycles`, `float pilot_amplitude_v`) to a `k_msgq` using `K_NO_WAIT`.
  - Queue overflow increments `drop_count` and immediately trips `FAULT`.
- **ISR-Safe Fast-Fail `FAULT` Cutoff**:
  - `pilot_mux_gpio_disconnect_all()` / `ops->disconnect_all()` acquires a `k_mutex` and MUST NOT be called from ISR.
  - On any ISR fault (ADC callback `error < 0`, `set_pilot_code() < 0`, DSP status `!= DSP_OK`, or `k_msgq_put != 0`), latch an atomic fault errno (`-EBADMSG` / `-EOVERFLOW` / `-EIO` / driver errno), immediately drive all 8 `pilot-sw-gpios` pins LOW via `gpio_pin_set_raw()` directly in ISR, and submit a static `k_work` to call `ops->disconnect_all()`, `bias_ads131m02_stop()`, and any registered fault callback in thread context.
- **Safe `SPI1` DAC Arbitration for `5 Hz` `VA` DC Updates**:
  - Provide a thread-context API (`rt_pipeline_set_dc_code(uint8_t dc, uint16_t code)`) that synchronizes with the `DRDY#` post-pilot window when streaming (waiting safely with timeout and faulting on DMA contention/timeout) or calls `ops->set_dc_code()` directly when stopped, preventing `-EBUSY` collisions with `64 kSPS` `DITHER1` DMA.
- **Block-Boundary Pilot Amplitude Updates**:
  - Provide `rt_pipeline_set_pilot_amplitude(float amp_v)` to stage a new `DITHER1` amplitude and apply it cleanly at the boundary of the next `1280`-sample block without phase discontinuity.
- **Testability / Mockability**:
  - Allow injecting `const analog_board_ops_t *` (defaulting to `analog_board_get_ops()`) and expose a pure sample-ingestion / fault-injection entry point so `p4_cli_and_ztest` can verify `rt_pipeline` deterministically with mocks.

## Control Thread & Calibration Service Contract (p4_ctrl_and_cal)
- **Calibration Persistence (`cal_service.{c,h}`)**:
  - `struct bias_eeprom_descriptor_v1` occupies `U21` bytes `0..147` via `ops->load_calibration` / `ops->store_calibration`.
  - Phase 4 Single-MZM affine calibration (`ctrl_calibration_t`: `fit.o[2]`, `fit.m[2][2]`, `null_v`, `vpi_v`, `cal_pilot_v`, `has_obs_dc_seed`, `obs_dc_seed`) is serialized in a CRC32-protected `cal_mzm_record_v1` stored at `U21` page-aligned offset `160` (`<= 96 B` inside `160..255`), written in `8 B` pages with `5 ms` write-cycle delay under `bias_i2c1_mutex`.
  - `cal_service_save()` syncs `descriptor.channels[0].vpi_uv` and stores both `descriptor` and `cal_mzm_record_v1`. `cal_service_load()` validates both CRCs and finite values (fast-fail on invalid/empty EEPROM without silently synthesizing fake calibration).
  - Supports injectable storage ops (`cal_storage_ops_t`) for deterministic `ztest` mocking without hardware I2C.
  - Provides `cal_service_voltage_to_dac_code()` and `cal_service_voltage_bounds()` using `channels[0].subtractor_gain_ppm` and `subtractor_offset_uv` (`DAC uV = (V*1e6 - offset_uv) * 1e6 / gain_ppm`, `0..2.5 V` -> `0..65535`; rejects `gain_ppm == 0` with `-EINVAL`).
- **Two-Pass `O(1)`-RAM Calibration & `scan vpi`**:
  - Uses `ctrl_bias_scan_t` (`< 1 KiB`) with `ctrl_bias_scan_add()`, `ctrl_bias_scan_begin_fit()`, and `ctrl_bias_scan_finish()` over `[min_v, max_v]` (settling + measurement windows per step).
  - `scan vpi` executes Pass 1 only (`dsp_vpi_tracker_t`); `cal bias` executes Pass 1 -> `ctrl_bias_scan_begin_fit()` -> Pass 2 (`dsp_online_lsq`) -> `ctrl_bias_scan_finish()`.
- **5 Hz Closed-Loop Control Thread (`ctrl_thread.{c,h}`)**:
  - Runs at Zephyr priority `5` with `2560 B` stack (`K_THREAD_STACK_DEFINE`).
  - Consumes `10` blocks (`1280` samples each at `50 Hz`) from `rt_pipeline_get_block()` into `5 Hz` (`dt = 0.2 s`) updates (or uses `ctrl_bias_engine_t` / block-averaging matching `ctrl_bias_engine.c` coherent I/Q averaging before `ctrl_mzm_strategy`).
  - Exposes thread-safe APIs for `cli_service` and `main.c`: `ctrl_thread_init()`, `ctrl_thread_start()`, `ctrl_thread_stop()`, `ctrl_thread_set_target(point, custom_deg)`, `ctrl_thread_start_cal(two_pass)`, `ctrl_thread_save_cal()`, `ctrl_thread_load_cal()`, `ctrl_thread_get_status()`.
  - On any `rt_pipeline` fault, DSP error, or DAC write error, immediately transitions to `CTRL_STATE_FAULT`, calls `ops->disconnect_all()` (all 8 `TS5A23159` switches LOW), and safely resets `VA` bias output.
  - Supports injectable `analog_board_ops_t`, `cal_storage_ops_t`, and manual `rt_pipeline` mode so `p4_cli_and_ztest` can test calibration + closed-loop locking under `ztest`.

## CLI Service, App Integration & Ztest Contract (p4_cli_and_ztest)
- **Non-Blocking UART CLI (`app/src/cli_service.{c,h}`)**:
  - Priority `8`, `2048 B` stack (`K_THREAD_STACK_DEFINE`), interrupt-driven `USART1` RX ring buffer + semaphore wakeup (zero busy-wait polling, non-blocking).
  - Commands: `help`, `status`, `cal bias`, `scan vpi`, `start`, `stop`, `target quad|min|max|custom <deg>`, `save`, `load`, plus `reinit` (explicit fault recovery) and `eeprom init` (writes default `20261110-MZM` descriptor for blank `U21` bring-up).
  - Formats floating-point telemetry via lightweight integer scaling (`mV` / `0.001` fixed-point) and parses `<deg>` without enabling `CONFIG_PICOLIBC_IO_FLOAT=y`, preserving Flash/RAM margins.
  - Exposes `cli_service_exec_line(const char *line, cli_write_fn_t out, void *ctx)` so `tests/ztest/` can verify every command deterministically without physical UART hardware.
- **Firmware Integration (`app/src/main.c` & `app/CMakeLists.txt`)**:
  - Builds `src/main.c`, `src/rt_pipeline.c`, `src/cal_service.c`, `src/ctrl_thread.c`, and `src/cli_service.c` (removing Phase 2 `diag_cli.c` from active app build so `USART1` IRQ ownership is exclusive to `cli_service`).
  - Initializes `analog_board`, `oled_lite`, `bias_keys`, `ctrl_thread_init(ops, NULL, false)`, and `cli_service_start()`, renders Phase 4 status on `OLED1`, and drives `LED1` heartbeat (`500 ms` normal / `150 ms` on `CTRL_STATE_FAULT` or boot error).
- **Mock-Backend Zephyr Integration Tests (`tests/ztest/`)**:
  - Zephyr `CONFIG_ZTEST=y` suite (`tests/ztest/CMakeLists.txt`, `tests/ztest/prj.conf`, `tests/ztest/src/...`) runnable on `qemu_cortex_m3` via `west build -b qemu_cortex_m3 -d build/ztest_p4 tests/ztest --pristine -t run`.
  - Links real `app/src/{rt_pipeline,cal_service,ctrl_thread,cli_service}.c` and `bias_dsp_control` (`lib/dsp/`, `lib/control/`) with mock `analog_board_ops_t`, mock `cal_storage_ops_t`, and stubbed `ads131m02`/`dac8568` symbols.
  - Verifies: (1) `rt_pipeline` `1280`-sample Goertzel/DC handoff, block-boundary pilot updates, queue-overflow `drop_count` & CRC `-EBADMSG` `crc_error_count` fault trips; (2) `cal_service` 7-page save/load round-trip, dual-CRC corruption rejection, and voltage-to-DAC bounds; (3) `ctrl_thread` two-pass `cal bias`, `scan vpi`, save/reload, closed-loop locking (`QUAD`/`MAX`/`MIN`/`CUSTOM`), and fault cutoff + `VA` reset; (4) `cli_service_exec_line()` command parsing/dispatch and error reporting.
