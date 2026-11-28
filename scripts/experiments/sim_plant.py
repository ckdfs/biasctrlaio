"""High-fidelity simulated MZM plant, UART CLI transport, and mock DG922 socket for dry-runs and tests."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
import socket
import numpy as np
from scipy.special import jv

from .common import DEFAULT_SAMPLE_RATE_HZ, DEFAULT_STANDARD_WINDOW_SAMPLES, wrap_rad


@dataclass
class MZMPlantConfig:
    """Physical parameters matching 20261110-MZM + STM32H503 dual-board bench."""

    vpi_dc_v: float = 5.382
    null_v_fw: float = -3.029            # Dark null MITP in firmware coord
    vpi_rf_v: float = 2.765              # Effective 20/25/50 MHz traveling-wave Vpi
    pilot_inj_gain: float = 0.09425      # 100 nF + 120k / 12k||4.7nF @ 1 kHz
    pilot_inj_phase_rad: float = -np.deg2rad(18.75)
    rx_lag_1k_rad: float = -np.deg2rad(82.0)
    rx_lag_2k_rad: float = -np.deg2rad(86.0)
    tia_gain_ohms: float = 47000.0
    i0_ua: float = 113.30
    visibility: float = 0.996
    dc_res_ohms: float = 3300.0
    dc_offset_mv: float = 0.5
    cross_coupling_12: float = 0.015     # Cross-coupling H1 -> H2 axis
    cross_coupling_21: float = -0.012    # Cross-coupling H2 -> H1 axis
    coherent_offset_h1_uv: complex = complex(40.0, 20.0)
    coherent_offset_h2_uv: complex = complex(-35.0, 25.0)
    noise_h1_uv: float = 3.5
    noise_h2_uv: float = 3.2
    noise_dc_mv: float = 0.15
    tau_lp_s: float = 0.01632            # 24 kOhm * 0.68 uF slow bias LPF
    tau_hp_s: float = 0.00083            # 8.3 kOhm * 100 nF AC TIA HPF
    vpi_drift_rel: float = 0.0           # e.g. +0.005 for +0.5% Vpi drift
    phase_drift_rad: float = 0.0         # slow thermal phase drift phi_0(t)
    rf_power_dbm: float | None = None
    rf_enabled: bool = False
    optical_power_scale: float = 1.0
    seed: int = 20261126


class SimulatedMZMPlant:
    """Analytical + transient-aware MZM physical model matching hardware polarity and units."""

    def __init__(self, cfg: MZMPlantConfig | None = None) -> None:
        self.cfg = cfg or MZMPlantConfig()
        self.rng = np.random.default_rng(self.cfg.seed)
        self.bias_v = 0.0
        self.settled_bias_v = 0.0
        self.seq = 0
        self.t_ms = 0
        self.pilot_v_cmd = 0.0
        self.last_step_dv = 0.0

    @property
    def null_v_thesis(self) -> float:
        return self.cfg.null_v_fw + self.cfg.vpi_dc_v

    @property
    def vpi_eff_v(self) -> float:
        return self.cfg.vpi_dc_v * (1.0 + self.cfg.vpi_drift_rel)

    def rf_j0(self) -> float:
        if not self.cfg.rf_enabled or self.cfg.rf_power_dbm is None:
            return 1.0
        watts = 10.0 ** ((self.cfg.rf_power_dbm - 30.0) / 10.0)
        v_pk = math.sqrt(2.0 * 50.0 * watts)
        m_rf = np.pi * v_pk / self.cfg.vpi_rf_v
        return float(jv(0, m_rf))

    def true_phase_thesis_rad(self, bias_v: float | None = None) -> float:
        vb = self.bias_v if bias_v is None else bias_v
        phi = np.pi * (vb - self.null_v_thesis) / self.vpi_eff_v + self.cfg.phase_drift_rad
        return float(wrap_rad(phi))

    def true_phase_fw_rad(self, bias_v: float | None = None) -> float:
        return float(wrap_rad(self.true_phase_thesis_rad(bias_v) - np.pi))

    def observe_block(
        self,
        bias_v: float,
        pilot_cmd_v: float,
        *,
        step_dv_v: float = 0.0,
        elapsed_after_step_s: float = 1.0,
    ) -> dict[str, float | int]:
        """Compute one 20 ms Goertzel block (H1/H2 in signed uV peak, DC in mV)."""
        phi = self.true_phase_thesis_rad(bias_v)
        # Nominal 0.10 V/V scaling in firmware vs actual 0.09425 V/V electrode gain.
        pilot_dac_v = pilot_cmd_v / 0.10
        v_el_pk = pilot_dac_v * self.cfg.pilot_inj_gain
        m = np.pi * v_el_pk / self.vpi_eff_v
        s_rf = self.rf_j0()
        p_scale = self.cfg.optical_power_scale
        i0_a = self.cfg.i0_ua * 1e-6 * p_scale

        # Dual-inverting analog chain (ch02.tex Eq. 2.13 / ch05.tex Eq. 5.3):
        # H1 retains negative sign (-sin phi), H2 flips negative (-cos phi).
        k1 = 2.0 * i0_a * self.cfg.visibility * s_rf * float(jv(1, m)) * self.cfg.tia_gain_ohms
        k2 = 2.0 * i0_a * self.cfg.visibility * s_rf * float(jv(2, m)) * self.cfg.tia_gain_ohms

        h1_ideal = -k1 * np.sin(phi)
        h2_ideal = -k2 * np.cos(phi)

        # Cross-coupling between harmonic projections before receiver lag.
        s1_phys = h1_ideal + self.cfg.cross_coupling_21 * k1 * np.cos(phi)
        s2_phys = h2_ideal + self.cfg.cross_coupling_12 * k2 * np.sin(phi)

        # Apply analog injection + 82 deg receiver phase rotation.
        psi1 = self.cfg.pilot_inj_phase_rad + self.cfg.rx_lag_1k_rad
        psi2 = 2.0 * self.cfg.pilot_inj_phase_rad + self.cfg.rx_lag_2k_rad
        h1_v = s1_phys * np.exp(1j * psi1) + self.cfg.coherent_offset_h1_uv * 1e-6
        h2_v = s2_phys * np.exp(1j * psi2) + self.cfg.coherent_offset_h2_uv * 1e-6

        # Step transient decay matching tau_LP = 16.32 ms and tau_HP = 0.83 ms
        # (calibrated to ch03.tex / B2: 0.5 V step after 50 ms wait gives ~-0.12 deg residual).
        transient_v = 0.0
        if abs(step_dv_v) > 1e-9:
            decay_lp = math.exp(-max(elapsed_after_step_s, 0.0) / self.cfg.tau_lp_s)
            transient_v = 0.95 * step_dv_v * decay_lp
            h1_v += 0.006 * step_dv_v * decay_lp * np.exp(1j * psi1)
            h2_v += 0.0008 * step_dv_v * decay_lp * np.exp(1j * psi2)

        h1_uv = h1_v * 1e6 + complex(self.rng.normal(0.0, self.cfg.noise_h1_uv), self.rng.normal(0.0, self.cfg.noise_h1_uv))
        h2_uv = h2_v * 1e6 + complex(self.rng.normal(0.0, self.cfg.noise_h2_uv), self.rng.normal(0.0, self.cfg.noise_h2_uv))

        dc_v = i0_a * (1.0 + self.cfg.visibility * s_rf * float(jv(0, m)) * np.cos(phi)) * self.cfg.dc_res_ohms
        dc_mv = dc_v * 1e3 + self.cfg.dc_offset_mv + float(self.rng.normal(0.0, self.cfg.noise_dc_mv))
        # Settled DC tracks LPF after a bias step.
        if abs(step_dv_v) > 1e-9:
            dc_mv -= (step_dv_v * 120.0) * math.exp(-max(elapsed_after_step_s, 0.0) / self.cfg.tau_lp_s)

        ch0_amp_mv = abs(h1_v) * 1e3 + abs(h2_v) * 1e3 + abs(transient_v) * 1e3
        ch0_amp_mv = min(ch0_amp_mv, 1200.0)
        ch1_clamped = float(np.clip(dc_mv, -1200.0, 1200.0))

        self.seq += 1
        self.t_ms += 20
        return {
            "seq": self.seq,
            "t_ms": self.t_ms,
            "bias_mv": bias_v * 1e3,
            "code": int(round(np.clip((bias_v + 10.0) / 20.0, 0.0, 1.0) * 65535.0)),
            "pilot_mv": pilot_cmd_v * 1e3,
            "h1_i_uv": int(round(h1_uv.real)),
            "h1_q_uv": int(round(h1_uv.imag)),
            "h2_i_uv": int(round(h2_uv.real)),
            "h2_q_uv": int(round(h2_uv.imag)),
            "dc_mv": dc_mv,
            "ch0_min_mv": -ch0_amp_mv,
            "ch0_max_mv": ch0_amp_mv,
            "ch1_min_mv": ch1_clamped - 0.3,
            "ch1_max_mv": ch1_clamped + 0.3,
        }


class SimulatedUARTTransport:
    """In-memory UART transport speaking exact Phase 7 CLI responses backed by SimulatedMZMPlant."""

    def __init__(self, plant: SimulatedMZMPlant | None = None, *, include_echo: bool = True) -> None:
        self.plant = plant or SimulatedMZMPlant()
        self.include_echo = include_echo
        self._rx_lines: deque[bytes] = deque()
        self.telemetry_enabled = False
        self.target_fw_deg = 90.0
        self.state = "IDLE"
        self.calibrated = True
        self.ctrl_params: dict[str, float] = {
            "pilot": 159.2,
            "wait": 3,
            "avg": 1,
            "gain": 0.25,
            "kp": 0.25,
            "ki": 0.85,
            "recal": 0,
            "rho_thresh": 0.25,
            "hold_ticks": 10,
        }
        self._tel_k = 0
        self._closed_loop_bias_v = self.plant.cfg.null_v_fw + 0.5 * self.plant.cfg.vpi_dc_v

    def _push(self, line: str) -> None:
        self._rx_lines.append((line + "\n").encode("utf-8"))

    def write(self, data: bytes) -> int:
        cmd = data.decode("utf-8").strip()
        if self.include_echo and not self.telemetry_enabled:
            self._push(f"> {cmd}")
        self._handle_command(cmd)
        return len(data)

    def readline(self) -> bytes:
        if not self._rx_lines and self.telemetry_enabled and self.state == "RUNNING":
            self._emit_tel_step()
        if self._rx_lines:
            return self._rx_lines.popleft()
        return b""

    def reset_input_buffer(self) -> None:
        self._rx_lines.clear()

    def close(self) -> None:
        pass

    def _emit_tel_step(self) -> None:
        self._tel_k += 1
        blk = self.plant.observe_block(self._closed_loop_bias_v, self.ctrl_params["pilot"] * 1e-3)
        phi_fw = self.plant.true_phase_fw_rad(self._closed_loop_bias_v)
        tgt_fw = float(wrap_rad(np.radians(self.target_fw_deg)))
        err_fw = float(wrap_rad(tgt_fw - phi_fw))
        # Closed-loop PI step in firmware simulation.
        s_rf = abs(self.plant.rf_j0())
        frozen = int(s_rf < 0.15)
        if not frozen:
            self._closed_loop_bias_v = float(
                np.clip(
                    self._closed_loop_bias_v + self.ctrl_params["gain"] * (self.plant.vpi_eff_v / np.pi) * err_fw,
                    -9.8,
                    9.8,
                )
            )
        locked = int((not frozen) and abs(err_fw) < 0.06)
        self._push(
            f"TEL,{self._tel_k},{blk['t_ms']},{blk['bias_mv']:.3f},{phi_fw:.3f},"
            f"{err_fw:.3f},{s_rf:.3f},{blk['h1_i_uv']},{blk['h1_q_uv']},"
            f"{blk['h2_i_uv']},{blk['h2_q_uv']},{blk['dc_mv']:.3f},{locked},{frozen},0,0,0"
        )

    def _handle_command(self, cmd: str) -> None:
        parts = cmd.split()
        if not parts:
            return
        if cmd == "hw info":
            self._push("SYSCLK=250000000 ADS131M02 ID=0x2200 STAT=0x0500 MODE=0x1111 CLOCK=0x0322")
            self._push("U21 desc_crc=0x1E71EE41 cal_status=0 cal_crc=0xA1B2C3D4")
            self._push("fw=P7 rev=sim board=biasctrl_h503 zephyr=4.5.0 build=dryrun")
            self._push("hw: ok")
            return
        if cmd == "status":
            tgt_name = "QUAD" if abs(self.target_fw_deg - 90.0) < 1e-3 else "CUSTOM"
            frozen_flag = int(abs(self.plant.rf_j0()) < 0.15)
            self._push(
                f"state={self.state} init=1 target={tgt_name}({self.target_fw_deg:.3f} deg) "
                f"cal={int(self.calibrated)} locked=0 hold_assist=0 frozen={frozen_flag} recal=0"
            )
            self._push(
                f"bias={int(round(self.plant.bias_v * 1e3))} mV scan_vpi=0 mV "
                f"vpi={int(round(self.plant.cfg.vpi_dc_v * 1e3))} mV "
                f"null={int(round(self.plant.cfg.null_v_fw * 1e3))} mV updates={self._tel_k} pass=0 step=0"
            )
            self._push("harmonics h1=0.000 mV@0.000 rad h2=0.000 mV@0.000 rad dc=400.000 mV")
            self._push("drops=0 crc_errs=0 fault=0 cleanup_err=0 load_err=0 rt_fault=0")
            return
        if parts[0] == "adc" and len(parts) == 3 and parts[1] == "stat":
            n = int(parts[2])
            if not (1000 <= n <= 1000000):
                self._push("adc: err=-34")
                return
            dt_cyc = 3906  # ~250e6 / 64000
            self._push(
                f"ADC_STAT,n={n},total_cyc={n * dt_cyc},fs_hz=64000.000,"
                f"dt_min_cyc={dt_cyc - 2},dt_max_cyc={dt_cyc + 2},drops=0,crc=0"
            )
            self._push("adc: ok")
            return
        if parts[0] == "adc" and len(parts) == 3 and parts[1] == "dump":
            n = int(parts[2])
            if not (1 <= n <= 16384):
                self._push("adc: err=-34")
                return
            pos = 0
            seg = 0
            while pos < n:
                cnt = min(DEFAULT_STANDARD_WINDOW_SAMPLES, n - pos)
                self._push(f"DUMP_SEG,{seg},{cnt},{1000 + pos * 3906}")
                samples = [int(round(self.plant.rng.normal(0.0, 25.0))) for _ in range(cnt)]
                for i in range(0, cnt, 16):
                    chunk = ",".join(str(v) for v in samples[i : i + 16])
                    self._push(f"DUMP_DATA,{chunk}")
                pos += cnt
                seg += 1
            self._push("adc: ok")
            return
        if parts[0] == "meas" and len(parts) == 5:
            bias_mv = float(parts[1])
            pilot_mv = float(parts[2])
            wait_b = int(parts[3])
            n_b = int(parts[4])
            bias_v = bias_mv * 1e-3
            dv = bias_v - self.plant.bias_v
            self.plant.bias_v = bias_v
            self.plant.pilot_v_cmd = pilot_mv * 1e-3
            self.state = "EXP_HOLD"
            self.plant.seq += wait_b
            self.plant.t_ms += wait_b * 20
            for idx in range(n_b):
                elapsed = (wait_b + idx + 0.5) * 0.020
                blk = self.plant.observe_block(bias_v, pilot_mv * 1e-3, step_dv_v=dv, elapsed_after_step_s=elapsed)
                self._push(
                    f"MEAS,{blk['seq']},{blk['t_ms']},{blk['bias_mv']:.3f},{blk['code']},{blk['pilot_mv']:.3f},"
                    f"{blk['h1_i_uv']},{blk['h1_q_uv']},{blk['h2_i_uv']},{blk['h2_q_uv']},"
                    f"{blk['dc_mv']:.3f},{blk['ch0_min_mv']:.1f},{blk['ch0_max_mv']:.1f},"
                    f"{blk['ch1_min_mv']:.1f},{blk['ch1_max_mv']:.1f},0,0"
                )
            self._push("meas: ok")
            return
        if parts[0] == "step" and len(parts) == 5:
            v0_v = float(parts[1]) * 1e-3
            v1_v = float(parts[2]) * 1e-3
            pilot_mv = float(parts[3])
            n_b = int(parts[4])
            self.state = "EXP_HOLD"
            self.plant.seq += 15
            self.plant.t_ms += 300
            dv = v1_v - v0_v
            self.plant.bias_v = v1_v
            for idx in range(n_b):
                elapsed = (idx + 0.5) * 0.020
                blk = self.plant.observe_block(v1_v, pilot_mv * 1e-3, step_dv_v=dv, elapsed_after_step_s=elapsed)
                self._push(
                    f"STEP,{blk['seq']},{blk['t_ms']},{blk['bias_mv']:.3f},{blk['code']},{blk['pilot_mv']:.3f},"
                    f"{blk['h1_i_uv']},{blk['h1_q_uv']},{blk['h2_i_uv']},{blk['h2_q_uv']},"
                    f"{blk['dc_mv']:.3f},{blk['ch0_min_mv']:.1f},{blk['ch0_max_mv']:.1f},"
                    f"{blk['ch1_min_mv']:.1f},{blk['ch1_max_mv']:.1f},0,0"
                )
            self._push("step: ok")
            return
        if parts[0] == "telemetry" and len(parts) == 2:
            self.telemetry_enabled = parts[1] == "on"
            self._push("telemetry: ok")
            return
        if cmd == "cal bias":
            self.calibrated = True
            self._push("cal bias: ok")
            return
        if parts[0] == "target":
            if parts[1] == "quad":
                self.target_fw_deg = 90.0
            elif parts[1] == "min":
                self.target_fw_deg = 0.0
            elif parts[1] == "max":
                self.target_fw_deg = 180.0
            elif parts[1] == "custom" and len(parts) == 3:
                self.target_fw_deg = float(parts[2])
            self._push("target: ok")
            return
        if cmd == "start":
            self.state = "RUNNING"
            tgt_fw = float(wrap_rad(np.radians(self.target_fw_deg)))
            self._closed_loop_bias_v = self.plant.cfg.null_v_fw + (tgt_fw / np.pi) * self.plant.cfg.vpi_dc_v + 0.1
            self.plant.bias_v = self._closed_loop_bias_v
            self._push("start: ok")
            return
        if cmd == "stop":
            self.state = "IDLE"
            self.plant.bias_v = 0.0
            self.plant.pilot_v_cmd = 0.0
            self._push("stop: ok")
            return
        if cmd in ("save", "load", "reinit"):
            self._push(f"{cmd}: ok")
            return
        if len(parts) == 4 and parts[0] == "ctrl" and parts[1] == "cfg":
            self.ctrl_params[parts[2]] = float(parts[3])
            self._push("ctrl cfg: ok")
            return
        self._push(f"Unknown command: {parts[0]} (type 'help')")


class SimulatedDG922Socket:
    """Mock TCP socket modeling DG922 Pro SCPI state (coupled to an optional SimulatedMZMPlant)."""

    def __init__(self, plant: SimulatedMZMPlant | None = None) -> None:
        self.plant = plant
        self.connected = False
        self.closed = False
        self.load = 9.9e37  # matches bench initial High-Z power-on state
        self.offset = 0.125 # matches bench initial non-zero offset
        self.freq = 20e6
        self.vpp = 0.632
        self.output = 0
        self.errors: list[str] = []
        self._rx_buf: deque[bytes] = deque()
        self.sent_commands: list[str] = []

    def settimeout(self, timeout: float) -> None:
        self.timeout = timeout

    def connect(self, addr) -> None:
        self.connected = True

    def close(self) -> None:
        self.closed = True

    def sendall(self, data: bytes) -> None:
        cmd = data.decode("ascii").strip()
        self.sent_commands.append(cmd)
        u = cmd.upper()
        if u == "*IDN?":
            self._rx_buf.append(b"RIGOL TECHNOLOGIES,DG922 Pro,DG9A000001,00.01.00\n")
        elif u in (":SYSTEM:ERROR?", ":SYST:ERR?"):
            self._rx_buf.append((self.errors.pop(0) if self.errors else '0,"No error"').encode("ascii") + b"\n")
        elif u.startswith((":OUTPUT1:LOAD ", ":OUTP1:LOAD ")):
            val = cmd.split()[-1].upper()
            self.load = 9.9e37 if val in ("INF", "INFINITY") else float(val)
        elif u in (":OUTPUT1:LOAD?", ":OUTP1:LOAD?"):
            self._rx_buf.append(f"{self.load:.6E}\n".encode("ascii"))
        elif u.startswith((":SOURCE1:VOLTAGE:OFFSET ", ":SOUR1:VOLT:OFFS ")):
            self.offset = float(cmd.split()[-1])
        elif u in (":SOURCE1:VOLTAGE:OFFSET?", ":SOUR1:VOLT:OFFS?"):
            self._rx_buf.append(f"{self.offset:.6E}\n".encode("ascii"))
        elif u.startswith((":SOURCE1:APPLY:SINUSOID ", ":SOUR1:APPL:SIN ")):
            args = [float(x) for x in cmd.split(None, 1)[1].split(",")]
            self.freq, self.vpp, self.offset = args[0], args[1], args[2]
        elif u in (":SOURCE1:FREQUENCY?", ":SOUR1:FREQ?"):
            self._rx_buf.append(f"{self.freq:.6E}\n".encode("ascii"))
        elif u in (":SOURCE1:VOLTAGE?", ":SOUR1:VOLT?"):
            self._rx_buf.append(f"{self.vpp:.6E}\n".encode("ascii"))
        elif u.startswith((":OUTPUT1:STATE ", ":OUTP1:STAT ")):
            state_str = cmd.split()[-1].upper()
            self.output = 1 if state_str in ("ON", "1") else 0
            if self.plant is not None:
                self.plant.cfg.rf_enabled = bool(self.output)
                v_pk = 0.5 * self.vpp
                watts = (v_pk ** 2) / (2.0 * 50.0)
                self.plant.cfg.rf_power_dbm = 10.0 * math.log10(max(watts, 1e-12)) + 30.0
        elif u in (":OUTPUT1:STATE?", ":OUTP1:STAT?"):
            self._rx_buf.append(f"{self.output}\n".encode("ascii"))
        else:
            self.errors.append(f'-113,"Undefined header: {cmd}"')

    def recv(self, bufsize: int) -> bytes:
        if self._rx_buf:
            return self._rx_buf.popleft()
        return b""
