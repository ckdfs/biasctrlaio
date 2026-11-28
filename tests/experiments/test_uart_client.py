"""Unit tests for uart_client.py wire protocol parsing and fast-fail checks."""

from __future__ import annotations

from collections import deque
from pathlib import Path
import unittest
from unittest.mock import patch

from scripts.experiments.common import fw_to_thesis_rad
from scripts.experiments.uart_client import (
    FirmwareCommandError,
    TelemetryIntegrityError,
    UARTClient,
    UARTClientError,
    parse_meas_row,
    parse_tel_row,
    pyocd_reset,
)


class ScriptedTransport:
    def __init__(self, responses: dict[str, list[str]]) -> None:
        self.responses = responses
        self.rx: deque[bytes] = deque()
        self.sent: list[str] = []

    def write(self, data: bytes) -> int:
        cmd = data.decode("utf-8").strip()
        self.sent.append(cmd)
        if cmd not in self.responses:
            raise AssertionError(f"Unexpected command {cmd!r}")
        for line in self.responses[cmd]:
            self.rx.append((line + "\n").encode("utf-8"))
        return len(data)

    def readline(self) -> bytes:
        return self.rx.popleft() if self.rx else b""

    def reset_input_buffer(self) -> None:
        self.rx.clear()

    def close(self) -> None:
        pass


class TestUARTClient(unittest.TestCase):
    def test_parse_meas_and_step_uv_units(self) -> None:
        line = "MEAS,3,1420,-340.000,31654,159.200,-250123,120456,5720,-311,418.561,-440.2,441.1,418.1,419.0,0,0"
        row = parse_meas_row(line)
        self.assertEqual(row.kind, "MEAS")
        self.assertEqual(row.seq, 3)
        self.assertEqual(row.t_ms, 1420)
        self.assertAlmostEqual(row.bias_mv, -340.0)
        self.assertEqual(row.code, 31654)
        self.assertAlmostEqual(row.pilot_mv, 159.2)
        self.assertEqual(row.h1_i_uv, -250123)
        self.assertEqual(row.h1_q_uv, 120456)
        self.assertEqual(row.h2_i_uv, 5720)
        self.assertEqual(row.h2_q_uv, -311)
        self.assertAlmostEqual(row.h1_complex_v.real, -0.250123, places=9)
        self.assertAlmostEqual(row.h1_complex_mv.imag, 120.456, places=6)
        self.assertAlmostEqual(row.h2_complex_v.real, 0.005720, places=9)
        self.assertAlmostEqual(row.h2_complex_v.imag, -0.000311, places=9)
        self.assertAlmostEqual(row.dc_mv, 418.561)
        self.assertAlmostEqual(row.ch0_min_mv, -440.2)
        self.assertAlmostEqual(row.ch1_max_mv, 419.0)

        step_line = line.replace("MEAS,", "STEP,", 1)
        srow = parse_meas_row(step_line)
        self.assertEqual(srow.kind, "STEP")

    def test_parse_tel_and_integrity_fast_fail(self) -> None:
        tel = parse_tel_row("TEL,46,9200,-340.000,1.571,0.000,1.000,-355000,12000,311,-150,418.561,1,0,2,0,0")
        self.assertEqual(tel.k, 46)
        self.assertTrue(tel.locked)
        self.assertFalse(tel.frozen)
        self.assertEqual(tel.recal_count, 2)
        self.assertAlmostEqual(tel.phi_hat_fw_rad, 1.571)
        self.assertAlmostEqual(tel.phi_hat_thesis_rad, fw_to_thesis_rad(1.571), places=9)
        self.assertAlmostEqual(tel.h1_complex_v.real, -0.355)

        with self.assertRaises(TelemetryIntegrityError):
            parse_meas_row("MEAS,1,20,0.000,32768,159.200,0,0,0,0,0.000,0.0,0.0,0.0,0.0,1,0")
        with self.assertRaises(TelemetryIntegrityError):
            parse_tel_row("TEL,1,20,0.000,0.000,0.000,1.000,0,0,0,0,0.000,1,0,0,0,1")

    def test_commands_hw_adc_meas_status_and_errors(self) -> None:
        tr = ScriptedTransport(
            {
                "hw info": [
                    "> hw info",
                    "SYSCLK=250000000 ADS131M02 ID=0x2200 STAT=0x0500 MODE=0x1111 CLOCK=0x0322",
                    "U21 desc_crc=0x1E71EE41 cal_status=0 cal_crc=0x1234ABCD",
                    "fw=P7 rev=abc board=biasctrl_h503 zephyr=4.5.0 build=1",
                    "hw: ok",
                ],
                "adc stat 100000": [
                    "ADC_STAT,n=100000,total_cyc=390625000,fs_hz=64000.000,dt_min_cyc=3905,dt_max_cyc=3908,drops=0,crc=0",
                    "adc: ok",
                ],
                "adc dump 18": [
                    "DUMP_SEG,0,18,12345",
                    "DUMP_DATA," + ",".join(str(i) for i in range(16)),
                    "DUMP_DATA,16,17",
                    "adc: ok",
                ],
                "meas -340.000 159.200 3 2": [
                    "TEL,99,500,-340.000,1.571,0.000,1.000,100,200,300,400,418.000,1,0,0,0,0",
                    "MEAS,10,200,-340.000,31654,159.200,-1000,2000,300,-400,418.100,-10.0,10.0,417.9,418.3,0,0",
                    "MEAS,11,220,-340.000,31654,159.200,-1005,2005,305,-405,418.200,-10.1,10.1,417.9,418.4,0,0",
                    "meas: ok",
                ],
                "status": [
                    "state=IDLE init=1 target=QUAD(90.000 deg) cal=1 locked=0 hold_assist=0",
                    "bias=0 mV scan_vpi=0 mV vpi=5382 mV null=-3029 mV updates=0 pass=0 step=0",
                    "harmonics h1=0.000 mV@0.000 rad h2=0.000 mV@0.000 rad dc=400.000 mV",
                    "drops=0 crc_errs=0 fault=0 cleanup_err=0 load_err=0 rt_fault=0",
                ],
                "ctrl cfg wait 3": ["ctrl cfg: ok"],
                "target custom 135.0000": ["target: err=-16"],
            }
        )
        client = UARTClient(transport=tr)

        hw = client.hw_info()
        self.assertEqual(hw.sysclk_hz, 250_000_000)
        self.assertEqual(hw.adc_clock, 0x0322)
        self.assertEqual(hw.desc_crc, 0x1E71EE41)

        stat = client.adc_stat(100_000)
        self.assertEqual(stat.fs_hz, 64000.0)

        dump = client.adc_dump(18)
        self.assertEqual(dump.total_samples, 18)
        self.assertEqual(dump.samples, tuple(range(18)))

        rows = client.meas(-340.0, 159.2, 3, 2)
        self.assertEqual(len(rows), 2)
        self.assertEqual(len(client.telemetry_backlog), 1)

        st = client.status()
        self.assertEqual(st.vpi_mv, 5382)

        client.ctrl_cfg("wait", 3)

        with self.assertRaises(FirmwareCommandError) as ctx:
            client.target("custom", 135.0)
        self.assertEqual(ctx.exception.errno, -16)

    def test_timeout_and_seq_gap_fail_fast(self) -> None:
        tr = ScriptedTransport(
            {
                "meas 0.000 0.000 0 2": [
                    "MEAS,1,20,0.000,32768,0.000,0,0,0,0,0.000,0.0,0.0,0.0,0.0,0,0",
                    "MEAS,3,60,0.000,32768,0.000,0,0,0,0,0.000,0.0,0.0,0.0,0.0,0,0",
                    "meas: ok",
                ],
                "hw info": ["SYSCLK=250000000 ADS131M02 ID=0x2200 STAT=0x0500 MODE=0x1111 CLOCK=0x0322"],
            }
        )
        client = UARTClient(transport=tr, default_timeout_s=0.05)
        with self.assertRaises(TelemetryIntegrityError):
            client.meas(0.0, 0.0, 0, 2)
        with self.assertRaises(TimeoutError):
            client.hw_info()

    def test_pyocd_reset_command(self) -> None:
        with patch("scripts.experiments.uart_client.subprocess.run") as mock_run:
            with patch.object(Path, "is_file", return_value=True):
                pyocd_reset(Path("/workspace/biasctrlaio"))
                args = mock_run.call_args[0][0]
                self.assertTrue(args[0].endswith(".venv/bin/pyocd"))
                self.assertEqual(args[1:], ["reset", "-t", "stm32h503cbtx"])

    def test_t2_status_with_frozen_recal_and_ctrl_cfg_error(self) -> None:
        tr = ScriptedTransport(
            {
                "status": [
                    "state=RUNNING init=1 target=CUSTOM(135.000 deg) cal=1 locked=1 hold_assist=0 frozen=1 recal=2",
                    "bias=-120 mV scan_vpi=5400 mV vpi=5382 mV null=-3029 mV updates=42 pass=2 step=8",
                    "harmonics h1=12.345 mV@0.123 rad h2=6.789 mV@1.456 rad dc=410.000 mV",
                    "drops=0 crc_errs=0 fault=0 cleanup_err=0 load_err=0 rt_fault=0",
                ],
                "ctrl cfg hold_ticks 9999": ["ctrl cfg: err=-34"],
            }
        )
        client = UARTClient(transport=tr)
        st = client.status()
        self.assertTrue(st.frozen)
        self.assertEqual(st.recal_count, 2)
        self.assertEqual(st.target_deg, 135.0)
        with self.assertRaises(FirmwareCommandError) as ctx:
            client.ctrl_cfg("hold_ticks", 9999)
        self.assertEqual(ctx.exception.errno, -34)

    def test_cal_bias_waits_for_idle(self) -> None:
        class CalSequenceTransport(ScriptedTransport):
            def __init__(self) -> None:
                super().__init__({"cal bias": ["cal bias: ok"]})
                self.status_calls = 0

            def write(self, data: bytes) -> int:
                cmd = data.decode("utf-8").strip()
                self.sent.append(cmd)
                if cmd == "cal bias":
                    self.rx.append(b"cal bias: ok\n")
                    return len(data)
                if cmd == "status":
                    self.status_calls += 1
                    st_name = "CAL" if self.status_calls == 1 else "IDLE"
                    cal_flag = 0 if self.status_calls == 1 else 1
                    for ln in (
                        f"state={st_name} init=1 target=QUAD(90.000 deg) cal={cal_flag} locked=0 hold_assist=0 frozen=0 recal=0",
                        "bias=0 mV scan_vpi=5380 mV vpi=5382 mV null=-3029 mV updates=10 pass=2 step=0",
                        "harmonics h1=0.000 mV@0.000 rad h2=0.000 mV@0.000 rad dc=400.000 mV",
                        "drops=0 crc_errs=0 fault=0 cleanup_err=0 load_err=0 rt_fault=0",
                    ):
                        self.rx.append((ln + "\n").encode("utf-8"))
                    return len(data)
                raise AssertionError(f"Unexpected cmd {cmd!r}")

        client = UARTClient(transport=CalSequenceTransport())
        st = client.cal_bias(timeout_s=5.0, poll_interval_s=0.0)
        self.assertIsNotNone(st)
        assert st is not None
        self.assertEqual(st.state, "IDLE")
        self.assertTrue(st.calibrated)


if __name__ == "__main__":
    unittest.main()
