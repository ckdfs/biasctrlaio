"""Deterministic UART CLI client and strict wire parser for Phase 7 firmware."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
import subprocess
import time
from typing import Protocol

from .common import fw_to_thesis_rad, repo_root

DEFAULT_SERIAL_PORT = "/dev/cu.usbmodem102"
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT_S = 5.0

_TERMINATOR_RE = re.compile(
    r"^(hw|adc|meas|step|telemetry|cal bias|scan vpi|target|start|stop|save|load|reinit|eeprom init|ctrl(?:\s+cfg)?):\s*(.+)$"
)
_ERR_RE = re.compile(r"\berr=(-?\d+)\b")
_ADC_STAT_RE = re.compile(
    r"^ADC_STAT,n=(\d+),total_cyc=(\d+),fs_hz=([0-9.]+),dt_min_cyc=(\d+),dt_max_cyc=(\d+),drops=(\d+),crc=(\d+)$"
)


class UARTClientError(RuntimeError):
    """Base fast-fail error for UART communication and parsing."""


class FirmwareCommandError(UARTClientError):
    """Raised when the firmware reports a negative err= code."""

    def __init__(self, command: str, errno: int, lines: list[str]) -> None:
        self.command = command
        self.errno = errno
        self.lines = lines
        super().__init__(f"Firmware command '{command}' failed with err={errno}: {lines}")


class TelemetryIntegrityError(UARTClientError):
    """Raised on non-zero drops, CRC errors, or sequence discontinuity."""


class SerialTransport(Protocol):
    """Minimal byte-line transport abstraction for hardware or mock testing."""

    def write(self, data: bytes) -> int: ...
    def readline(self) -> bytes: ...
    def reset_input_buffer(self) -> None: ...
    def close(self) -> None: ...


class PySerialTransport:
    """Thin pyserial wrapper for 115200 8N1 UART communication."""

    def __init__(self, port: str = DEFAULT_SERIAL_PORT, baudrate: int = DEFAULT_BAUDRATE, timeout: float = 0.25) -> None:
        import serial  # lazy import so tests using MockTransport don't fail if optional

        self._ser = serial.Serial(port=port, baudrate=baudrate, bytesize=8, parity="N", stopbits=1, timeout=timeout)

    def write(self, data: bytes) -> int:
        return int(self._ser.write(data))

    def readline(self) -> bytes:
        return bytes(self._ser.readline())

    def reset_input_buffer(self) -> None:
        self._ser.reset_input_buffer()

    def close(self) -> None:
        self._ser.close()


@dataclass(frozen=True)
class HwInfoResult:
    sysclk_hz: int
    adc_id: int
    adc_stat: int
    adc_mode: int
    adc_clock: int
    desc_crc: int
    cal_status: int
    cal_crc: int
    fw: str
    rev: str
    board: str
    zephyr: str
    build: str
    raw_lines: tuple[str, ...]


@dataclass(frozen=True)
class AdcStatResult:
    n: int
    total_cyc: int
    fs_hz: float
    dt_min_cyc: int
    dt_max_cyc: int
    drops: int
    crc: int


@dataclass(frozen=True)
class AdcDumpSegment:
    index: int
    count: int
    first_cycles: int
    samples: tuple[int, ...]


@dataclass(frozen=True)
class AdcDumpResult:
    total_samples: int
    segments: tuple[AdcDumpSegment, ...]
    samples: tuple[int, ...]


@dataclass(frozen=True)
class MeasRow:
    kind: str  # "MEAS" or "STEP"
    seq: int
    t_ms: int
    bias_mv: float
    code: int
    pilot_mv: float
    h1_i_uv: int
    h1_q_uv: int
    h2_i_uv: int
    h2_q_uv: int
    dc_mv: float
    ch0_min_mv: float
    ch0_max_mv: float
    ch1_min_mv: float
    ch1_max_mv: float
    drops: int
    crc: int

    @property
    def h1_complex_v(self) -> complex:
        return complex(self.h1_i_uv * 1e-6, self.h1_q_uv * 1e-6)

    @property
    def h2_complex_v(self) -> complex:
        return complex(self.h2_i_uv * 1e-6, self.h2_q_uv * 1e-6)

    @property
    def h1_complex_mv(self) -> complex:
        return complex(self.h1_i_uv * 1e-3, self.h1_q_uv * 1e-3)

    @property
    def h2_complex_mv(self) -> complex:
        return complex(self.h2_i_uv * 1e-3, self.h2_q_uv * 1e-3)

    @property
    def dc_v(self) -> float:
        return self.dc_mv * 1e-3


@dataclass(frozen=True)
class TelemetryRow:
    k: int
    t_ms: int
    bias_mv: float
    phi_hat_fw_rad: float
    phi_hat_thesis_rad: float
    err_rad: float
    radius: float
    h1_i_uv: int
    h1_q_uv: int
    h2_i_uv: int
    h2_q_uv: int
    dc_mv: float
    locked: bool
    frozen: bool
    recal_count: int
    drops: int
    crc: int

    @property
    def h1_complex_v(self) -> complex:
        return complex(self.h1_i_uv * 1e-6, self.h1_q_uv * 1e-6)

    @property
    def h2_complex_v(self) -> complex:
        return complex(self.h2_i_uv * 1e-6, self.h2_q_uv * 1e-6)


@dataclass(frozen=True)
class StatusResult:
    state: str
    init_ok: bool
    target: str
    target_deg: float
    calibrated: bool
    locked: bool
    hold_assist: bool
    bias_mv: int
    scan_vpi_mv: int
    vpi_mv: int
    null_mv: int
    updates: int
    scan_pass: int
    scan_step: int
    h1_mv: float
    h1_rad: float
    h2_mv: float
    h2_rad: float
    dc_mv: float
    drops: int
    crc_errs: int
    fault: int
    cleanup_err: int
    load_err: int
    rt_fault: int
    frozen: bool = False
    recal_count: int = 0


def parse_meas_row(line: str) -> MeasRow:
    """Parse MEAS or STEP CSV line strictly; fast-fail on malformed row or drops/crc > 0."""
    parts = line.strip().split(",")
    if len(parts) != 17 or parts[0] not in ("MEAS", "STEP"):
        raise UARTClientError(f"Malformed MEAS/STEP row: {line!r}")
    row = MeasRow(
        kind=parts[0],
        seq=int(parts[1]),
        t_ms=int(parts[2]),
        bias_mv=float(parts[3]),
        code=int(parts[4]),
        pilot_mv=float(parts[5]),
        h1_i_uv=int(parts[6]),
        h1_q_uv=int(parts[7]),
        h2_i_uv=int(parts[8]),
        h2_q_uv=int(parts[9]),
        dc_mv=float(parts[10]),
        ch0_min_mv=float(parts[11]),
        ch0_max_mv=float(parts[12]),
        ch1_min_mv=float(parts[13]),
        ch1_max_mv=float(parts[14]),
        drops=int(parts[15]),
        crc=int(parts[16]),
    )
    if row.drops > 0 or row.crc > 0:
        raise TelemetryIntegrityError(f"Integrity failure in {row.kind} seq={row.seq}: drops={row.drops}, crc={row.crc}")
    return row


def parse_tel_row(line: str) -> TelemetryRow:
    """Parse TEL CSV line strictly; fast-fail on malformed row or drops/crc > 0."""
    parts = line.strip().split(",")
    if len(parts) != 17 or parts[0] != "TEL":
        raise UARTClientError(f"Malformed TEL row: {line!r}")
    phi_fw = float(parts[4])
    row = TelemetryRow(
        k=int(parts[1]),
        t_ms=int(parts[2]),
        bias_mv=float(parts[3]),
        phi_hat_fw_rad=phi_fw,
        phi_hat_thesis_rad=float(fw_to_thesis_rad(phi_fw)),
        err_rad=float(parts[5]),
        radius=float(parts[6]),
        h1_i_uv=int(parts[7]),
        h1_q_uv=int(parts[8]),
        h2_i_uv=int(parts[9]),
        h2_q_uv=int(parts[10]),
        dc_mv=float(parts[11]),
        locked=bool(int(parts[12])),
        frozen=bool(int(parts[13])),
        recal_count=int(parts[14]),
        drops=int(parts[15]),
        crc=int(parts[16]),
    )
    if row.drops > 0 or row.crc > 0:
        raise TelemetryIntegrityError(f"Integrity failure in TEL k={row.k}: drops={row.drops}, crc={row.crc}")
    return row


def parse_adc_stat_line(line: str) -> AdcStatResult:
    m = _ADC_STAT_RE.match(line.strip())
    if not m:
        raise UARTClientError(f"Malformed ADC_STAT line: {line!r}")
    res = AdcStatResult(
        n=int(m.group(1)),
        total_cyc=int(m.group(2)),
        fs_hz=float(m.group(3)),
        dt_min_cyc=int(m.group(4)),
        dt_max_cyc=int(m.group(5)),
        drops=int(m.group(6)),
        crc=int(m.group(7)),
    )
    if res.drops > 0 or res.crc > 0:
        raise TelemetryIntegrityError(f"Integrity failure in ADC_STAT: drops={res.drops}, crc={res.crc}")
    return res


def parse_hw_info_lines(lines: list[str]) -> HwInfoResult:
    sys_line = next((ln for ln in lines if ln.startswith("SYSCLK=")), None)
    u21_line = next((ln for ln in lines if ln.startswith("U21 ")), None)
    fw_line = next((ln for ln in lines if ln.startswith("fw=")), None)
    if sys_line is None or u21_line is None or fw_line is None:
        raise UARTClientError(f"Incomplete hw info output: {lines}")

    m1 = re.match(
        r"^SYSCLK=(\d+)\s+ADS131M02\s+ID=0x([0-9A-Fa-f]+)\s+STAT=0x([0-9A-Fa-f]+)\s+MODE=0x([0-9A-Fa-f]+)\s+CLOCK=0x([0-9A-Fa-f]+)$",
        sys_line,
    )
    m2 = re.match(
        r"^U21\s+desc_crc=0x([0-9A-Fa-f]+)\s+cal_status=(-?\d+)\s+cal_crc=0x([0-9A-Fa-f]+)$",
        u21_line,
    )
    m3 = re.match(
        r"^fw=(\S+)\s+rev=(\S+)\s+board=(\S+)\s+zephyr=(\S+)\s+build=(\S+)$",
        fw_line,
    )
    if not m1 or not m2 or not m3:
        raise UARTClientError(f"Malformed hw info lines: {lines}")
    return HwInfoResult(
        sysclk_hz=int(m1.group(1)),
        adc_id=int(m1.group(2), 16),
        adc_stat=int(m1.group(3), 16),
        adc_mode=int(m1.group(4), 16),
        adc_clock=int(m1.group(5), 16),
        desc_crc=int(m2.group(1), 16),
        cal_status=int(m2.group(2)),
        cal_crc=int(m2.group(3), 16),
        fw=m3.group(1),
        rev=m3.group(2),
        board=m3.group(3),
        zephyr=m3.group(4),
        build=m3.group(5),
        raw_lines=tuple(lines),
    )


def parse_status_lines(lines: list[str]) -> StatusResult:
    l1 = next((ln for ln in lines if ln.startswith("state=")), None)
    l2 = next((ln for ln in lines if ln.startswith("bias=")), None)
    l3 = next((ln for ln in lines if ln.startswith("harmonics ")), None)
    l4 = next((ln for ln in lines if ln.startswith("drops=")), None)
    if not all((l1, l2, l3, l4)):
        raise UARTClientError(f"Incomplete status output: {lines}")

    m1 = re.match(
        r"^state=(\w+)\s+init=(\d+)\s+target=(\w+)\(([-0-9.naninf]+)\s+deg\)\s+cal=(\d+)\s+locked=(\d+)\s+hold_assist=(\d+)(?:\s+frozen=(\d+)\s+recal=(\d+))?$",
        l1,
    )
    m2 = re.match(
        r"^bias=(-?\d+)\s+mV\s+scan_vpi=(-?\d+)\s+mV\s+vpi=(-?\d+)\s+mV\s+null=(-?\d+)\s+mV\s+updates=(\d+)\s+pass=(\d+)\s+step=(\d+)$",
        l2,
    )
    m3 = re.match(
        r"^harmonics\s+h1=([-0-9.nan]+)\s+mV@([-0-9.nan]+)\s+rad\s+h2=([-0-9.nan]+)\s+mV@([-0-9.nan]+)\s+rad\s+dc=([-0-9.nan]+)\s+mV$",
        l3,
    )
    m4 = re.match(
        r"^drops=(\d+)\s+crc_errs=(\d+)\s+fault=(-?\d+)\s+cleanup_err=(-?\d+)\s+load_err=(-?\d+)\s+rt_fault=(-?\d+)$",
        l4,
    )
    if not all((m1, m2, m3, m4)):
        raise UARTClientError(f"Malformed status output: {lines}")

    res = StatusResult(
        state=m1.group(1),
        init_ok=bool(int(m1.group(2))),
        target=m1.group(3),
        target_deg=float(m1.group(4)),
        calibrated=bool(int(m1.group(5))),
        locked=bool(int(m1.group(6))),
        hold_assist=bool(int(m1.group(7))),
        bias_mv=int(m2.group(1)),
        scan_vpi_mv=int(m2.group(2)),
        vpi_mv=int(m2.group(3)),
        null_mv=int(m2.group(4)),
        updates=int(m2.group(5)),
        scan_pass=int(m2.group(6)),
        scan_step=int(m2.group(7)),
        h1_mv=float(m3.group(1)),
        h1_rad=float(m3.group(2)),
        h2_mv=float(m3.group(3)),
        h2_rad=float(m3.group(4)),
        dc_mv=float(m3.group(5)),
        drops=int(m4.group(1)),
        crc_errs=int(m4.group(2)),
        fault=int(m4.group(3)),
        cleanup_err=int(m4.group(4)),
        load_err=int(m4.group(5)),
        rt_fault=int(m4.group(6)),
        frozen=bool(int(m1.group(8))) if m1.group(8) is not None else False,
        recal_count=int(m1.group(9)) if m1.group(9) is not None else 0,
    )
    if res.drops > 0 or res.crc_errs > 0:
        raise TelemetryIntegrityError(f"Integrity failure in status: drops={res.drops}, crc_errs={res.crc_errs}")
    return res


def pyocd_reset(
    project_root: Path | None = None,
    *,
    target: str = "stm32h503cbtx",
    timeout_s: float = 15.0,
) -> subprocess.CompletedProcess[str]:
    """Reset the STM32H503 target via .venv/bin/pyocd."""
    root = repo_root() if project_root is None else Path(project_root)
    pyocd_bin = root / ".venv" / "bin" / "pyocd"
    if not pyocd_bin.is_file():
        raise FileNotFoundError(f"pyocd executable not found at {pyocd_bin}")
    return subprocess.run(
        [str(pyocd_bin), "reset", "-t", target],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )


class UARTClient:
    """Synchronous strict-parsing CLI client over UART or an injectable transport."""

    def __init__(
        self,
        port: str = DEFAULT_SERIAL_PORT,
        *,
        baudrate: int = DEFAULT_BAUDRATE,
        default_timeout_s: float = DEFAULT_TIMEOUT_S,
        transport: SerialTransport | None = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.default_timeout_s = default_timeout_s
        self._transport = transport
        self._owns_transport = transport is None
        self.telemetry_backlog: list[TelemetryRow] = []

    def __enter__(self) -> "UARTClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        self.close()
        return False

    def connect(self) -> None:
        if self._transport is None:
            self._transport = PySerialTransport(self.port, baudrate=self.baudrate)
            self._owns_transport = True

    def close(self) -> None:
        if self._transport is not None and self._owns_transport:
            self._transport.close()
            self._transport = None

    def _require_transport(self) -> SerialTransport:
        if self._transport is None:
            raise UARTClientError("UARTClient is not connected")
        return self._transport

    def _readline_clean(self) -> str | None:
        tr = self._require_transport()
        raw = tr.readline()
        if not raw:
            return None
        text = raw.decode("utf-8", errors="replace").replace("\r", "").strip("\n")
        # Strip leading CLI prompt marker "> " if echoed on the same line.
        while text.startswith("> "):
            text = text[2:]
        return text.strip()

    def _send_line(self, command: str) -> None:
        tr = self._require_transport()
        payload = (command.strip() + "\n").encode("ascii")
        tr.write(payload)

    def _exec_terminated(self, command: str, *, timeout_s: float | None = None) -> list[str]:
        """Send a CLI command and collect lines until `<cmd>: ok` or `<cmd>: err=<n>`."""
        self._send_line(command)
        deadline = time.monotonic() + (self.default_timeout_s if timeout_s is None else timeout_s)
        lines: list[str] = []
        cmd_stripped = command.strip()

        while time.monotonic() < deadline:
            line = self._readline_clean()
            if line is None or line == "" or line == ">":
                continue
            if line == cmd_stripped:
                # Local echo of the command string.
                continue
            if line.startswith("TEL,"):
                self.telemetry_backlog.append(parse_tel_row(line))
                continue
            if line.startswith("Unknown command:") or line.startswith("line too long") or line.startswith("too many arguments"):
                m_err = _ERR_RE.search(line)
                errno = int(m_err.group(1)) if m_err else -22
                raise FirmwareCommandError(cmd_stripped, errno, lines + [line])

            m_term = _TERMINATOR_RE.match(line)
            if m_term:
                status_part = m_term.group(2).strip()
                m_err = _ERR_RE.search(status_part)
                if m_err:
                    errno = int(m_err.group(1))
                    raise FirmwareCommandError(cmd_stripped, errno, lines + [line])
                if status_part == "ok" or status_part.startswith("ok "):
                    lines.append(line)
                    return lines
            lines.append(line)

        raise TimeoutError(f"Timeout ({timeout_s or self.default_timeout_s}s) waiting for '{cmd_stripped}' completion; partial={lines}")

    def read_boot_banner(self, *, timeout_s: float = 5.0) -> list[str]:
        """Collect boot log lines up to `CLI ready` or timeout."""
        deadline = time.monotonic() + timeout_s
        lines: list[str] = []
        while time.monotonic() < deadline:
            line = self._readline_clean()
            if line is None or line == "" or line == ">":
                continue
            lines.append(line)
            if "CLI ready" in line:
                return lines
        raise TimeoutError(f"Timeout waiting for boot banner; captured={lines}")

    def hw_info(self) -> HwInfoResult:
        lines = self._exec_terminated("hw info")
        return parse_hw_info_lines(lines)

    def status(self, *, fail_on_fault: bool = False) -> StatusResult:
        self._send_line("status")
        deadline = time.monotonic() + self.default_timeout_s
        lines: list[str] = []
        while time.monotonic() < deadline:
            line = self._readline_clean()
            if line is None or line == "" or line == ">":
                continue
            if line == "status":
                continue
            if line.startswith("TEL,"):
                self.telemetry_backlog.append(parse_tel_row(line))
                continue
            lines.append(line)
            if line.startswith("drops="):
                res = parse_status_lines(lines)
                if fail_on_fault and (res.fault < 0 or res.rt_fault < 0):
                    raise FirmwareCommandError("status", res.fault or res.rt_fault, lines)
                return res
        raise TimeoutError(f"Timeout waiting for status response; partial={lines}")

    def adc_stat(self, n: int, *, timeout_s: float | None = None) -> AdcStatResult:
        if not (1000 <= n <= 1000000):
            raise ValueError(f"adc stat n out of range [1000, 1000000]: {n}")
        t_wait = max(self.default_timeout_s, n / 64000.0 + 3.0) if timeout_s is None else timeout_s
        lines = self._exec_terminated(f"adc stat {n}", timeout_s=t_wait)
        stat_line = next((ln for ln in lines if ln.startswith("ADC_STAT,")), None)
        if stat_line is None:
            raise UARTClientError(f"Missing ADC_STAT line in response: {lines}")
        return parse_adc_stat_line(stat_line)

    def adc_dump(self, n: int, *, timeout_s: float | None = None) -> AdcDumpResult:
        if not (1 <= n <= 16384):
            raise ValueError(f"adc dump n out of range [1, 16384]: {n}")
        t_wait = max(self.default_timeout_s, n * 0.002 + 5.0) if timeout_s is None else timeout_s
        lines = self._exec_terminated(f"adc dump {n}", timeout_s=t_wait)

        segments: list[AdcDumpSegment] = []
        all_samples: list[int] = []
        cur_idx: int | None = None
        cur_count = 0
        cur_cyc = 0
        cur_buf: list[int] = []

        for line in lines[:-1]:
            if line.startswith("DUMP_SEG,"):
                if cur_idx is not None:
                    if len(cur_buf) != cur_count:
                        raise UARTClientError(f"DUMP_SEG {cur_idx} expected {cur_count} samples, got {len(cur_buf)}")
                    segments.append(AdcDumpSegment(cur_idx, cur_count, cur_cyc, tuple(cur_buf)))
                parts = line.split(",")
                if len(parts) != 4:
                    raise UARTClientError(f"Malformed DUMP_SEG: {line!r}")
                cur_idx = int(parts[1])
                cur_count = int(parts[2])
                cur_cyc = int(parts[3])
                cur_buf = []
                if cur_idx != len(segments):
                    raise UARTClientError(f"Non-sequential DUMP_SEG index {cur_idx}, expected {len(segments)}")
            elif line.startswith("DUMP_DATA,"):
                if cur_idx is None:
                    raise UARTClientError(f"DUMP_DATA before DUMP_SEG: {line!r}")
                vals = [int(tok) for tok in line.split(",")[1:]]
                cur_buf.extend(vals)
                all_samples.extend(vals)
            else:
                raise UARTClientError(f"Unexpected line during adc dump: {line!r}")

        if cur_idx is not None:
            if len(cur_buf) != cur_count:
                raise UARTClientError(f"DUMP_SEG {cur_idx} expected {cur_count} samples, got {len(cur_buf)}")
            segments.append(AdcDumpSegment(cur_idx, cur_count, cur_cyc, tuple(cur_buf)))

        if len(all_samples) != n:
            raise UARTClientError(f"adc dump {n} returned {len(all_samples)} samples")
        return AdcDumpResult(total_samples=n, segments=tuple(segments), samples=tuple(all_samples))

    def meas(
        self,
        bias_mv: float,
        pilot_mv: float,
        wait_blocks: int,
        n_blocks: int,
        *,
        timeout_s: float | None = None,
    ) -> list[MeasRow]:
        if not (0.0 <= pilot_mv <= 250.0):
            raise ValueError(f"pilot_mv out of range [0, 250]: {pilot_mv}")
        if not (0 <= wait_blocks <= 250):
            raise ValueError(f"wait_blocks out of range [0, 250]: {wait_blocks}")
        if not (1 <= n_blocks <= 2000):
            raise ValueError(f"n_blocks out of range [1, 2000]: {n_blocks}")

        cmd = f"meas {bias_mv:.3f} {pilot_mv:.3f} {wait_blocks} {n_blocks}"
        t_wait = max(self.default_timeout_s, (wait_blocks + n_blocks) * 0.025 + 3.0) if timeout_s is None else timeout_s
        lines = self._exec_terminated(cmd, timeout_s=t_wait)
        rows = [parse_meas_row(ln) for ln in lines[:-1] if ln.startswith("MEAS,")]
        if len(rows) != n_blocks:
            raise UARTClientError(f"Expected {n_blocks} MEAS rows, got {len(rows)}: {lines}")
        for idx in range(1, len(rows)):
            if rows[idx].seq != rows[idx - 1].seq + 1:
                raise TelemetryIntegrityError(
                    f"Non-consecutive MEAS seq: {rows[idx - 1].seq} -> {rows[idx].seq}"
                )
        return rows

    def step(
        self,
        v0_mv: float,
        v1_mv: float,
        pilot_mv: float,
        n_blocks: int,
        *,
        timeout_s: float | None = None,
    ) -> list[MeasRow]:
        if not (0.0 <= pilot_mv <= 250.0):
            raise ValueError(f"pilot_mv out of range [0, 250]: {pilot_mv}")
        if not (1 <= n_blocks <= 200):
            raise ValueError(f"n_blocks out of range [1, 200]: {n_blocks}")

        cmd = f"step {v0_mv:.3f} {v1_mv:.3f} {pilot_mv:.3f} {n_blocks}"
        t_wait = max(self.default_timeout_s, (15 + n_blocks) * 0.025 + 3.0) if timeout_s is None else timeout_s
        lines = self._exec_terminated(cmd, timeout_s=t_wait)
        rows = [parse_meas_row(ln) for ln in lines[:-1] if ln.startswith("STEP,")]
        if len(rows) != n_blocks:
            raise UARTClientError(f"Expected {n_blocks} STEP rows, got {len(rows)}: {lines}")
        for idx in range(1, len(rows)):
            if rows[idx].seq != rows[idx - 1].seq + 1:
                raise TelemetryIntegrityError(
                    f"Non-consecutive STEP seq: {rows[idx - 1].seq} -> {rows[idx].seq}"
                )
        return rows

    def telemetry(self, enabled: bool) -> None:
        self._exec_terminated(f"telemetry {'on' if enabled else 'off'}")

    def read_telemetry_rows(self, duration_s: float, *, max_rows: int | None = None) -> list[TelemetryRow]:
        """Collect streaming TEL rows for up to duration_s seconds."""
        rows: list[TelemetryRow] = []
        if self.telemetry_backlog:
            rows.extend(self.telemetry_backlog)
            self.telemetry_backlog.clear()
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            if max_rows is not None and len(rows) >= max_rows:
                break
            line = self._readline_clean()
            if line is None or line == "" or line == ">":
                continue
            if line.startswith("TEL,"):
                rows.append(parse_tel_row(line))
            elif _ERR_RE.search(line):
                m_err = _ERR_RE.search(line)
                raise FirmwareCommandError("telemetry_stream", int(m_err.group(1)), [line])
        return rows

    def cal_bias(
        self,
        *,
        timeout_s: float = 260.0,
        wait_complete: bool = True,
        poll_interval_s: float = 1.0,
    ) -> StatusResult | None:
        self._exec_terminated("cal bias", timeout_s=min(self.default_timeout_s, timeout_s))
        if not wait_complete:
            return None
        deadline = time.monotonic() + timeout_s
        while True:
            st = self.status(fail_on_fault=True)
            if st.state == "IDLE":
                if not st.calibrated:
                    raise FirmwareCommandError("cal bias", st.fault or -61, ["CAL finished uncalibrated"])
                return st
            if st.state != "CAL":
                raise FirmwareCommandError("cal bias", st.fault or -16, [f"Unexpected state={st.state}"])
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Timed out ({timeout_s}s) waiting for cal bias completion; last={st}")
            time.sleep(poll_interval_s)

    def target(self, point: str, custom_deg: float | None = None) -> None:
        pt = point.lower()
        if pt in ("quad", "min", "max"):
            self._exec_terminated(f"target {pt}")
        elif pt == "custom":
            if custom_deg is None:
                raise ValueError("custom_deg is required when point='custom'")
            self._exec_terminated(f"target custom {custom_deg:.4f}")
        else:
            raise ValueError(f"Unsupported target point: {point}")

    def start(self) -> None:
        self._exec_terminated("start")

    def stop(self) -> None:
        self._exec_terminated("stop")

    def save(self) -> None:
        self._exec_terminated("save")

    def load(self) -> None:
        self._exec_terminated("load")

    def reinit(self) -> None:
        self._exec_terminated("reinit")

    def ctrl_cfg(self, key: str, value: float | int | str) -> None:
        valid_keys = {"pilot", "wait", "avg", "gain", "kp", "ki", "recal", "rho_thresh", "hold_ticks"}
        if key not in valid_keys:
            raise ValueError(f"Unknown ctrl cfg key '{key}', expected one of {sorted(valid_keys)}")
        self._exec_terminated(f"ctrl cfg {key} {value}")
