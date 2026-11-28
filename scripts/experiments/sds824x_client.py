"""Hardened TCP/SCPI client for Siglent SDS824X HD oscilloscope (192.168.99.157:5025)."""

from __future__ import annotations

import socket
import struct
import time
from typing import Callable

import numpy as np

DEFAULT_HOST = "192.168.99.157"
DEFAULT_PORT = 5025
DEFAULT_TIMEOUT_S = 10.0
CMD_SLEEP_S = 0.06
WAVEFORM_SLEEP_S = 0.15
HORI_DIVISIONS = 10.0

_TDIV_ENUM = (
    200e-12, 500e-12, 1e-9, 2e-9, 5e-9, 10e-9, 20e-9, 50e-9, 100e-9, 200e-9,
    500e-9, 1e-6, 2e-6, 5e-6, 10e-6, 20e-6, 50e-6, 100e-6, 200e-6, 500e-6,
    1e-3, 2e-3, 5e-3, 10e-3, 20e-3, 50e-3, 100e-3, 200e-3, 500e-3,
    1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0,
)

_COUPLING_MAP = {
    "DC": "D1M",
    "D1M": "D1M",
    "AC": "A1M",
    "A1M": "A1M",
    "GND": "GND",
}


class SDS824XError(RuntimeError):
    """Raised when SDS824X HD SCPI communication, acquisition state, or binary parsing fails."""


def build_wavedesc(
    *,
    point_num: int,
    vdiv: float,
    offset: float,
    code_per_div: float,
    adc_bit: int,
    interval: float,
    delay: float,
    tdiv_s: float,
    probe: float = 1.0,
) -> bytes:
    """Construct a valid 346-byte Siglent WAVEDESC descriptor (for tests and dry-runs)."""
    desc = bytearray(346)
    desc[0x00:0x08] = b"WAVEDESC"
    struct.pack_into("<hh", desc, 0x20, 1, 0)  # WORD, LSB
    struct.pack_into("<i", desc, 0x74, int(point_num))
    struct.pack_into("<f", desc, 0x9C, float(vdiv) / float(probe))
    struct.pack_into("<f", desc, 0xA0, float(offset) / float(probe))
    struct.pack_into("<f", desc, 0xA4, float(code_per_div))
    struct.pack_into("<h", desc, 0xAC, int(adc_bit))
    struct.pack_into("<f", desc, 0xB0, float(interval))
    struct.pack_into("<d", desc, 0xB4, float(delay))
    best_idx = min(range(len(_TDIV_ENUM)), key=lambda i: abs(_TDIV_ENUM[i] - float(tdiv_s)))
    struct.pack_into("<h", desc, 0x144, best_idx)
    struct.pack_into("<f", desc, 0x148, float(probe))
    return bytes(desc)


def parse_wavedesc(desc: bytes) -> dict[str, float | int | None]:
    """Parse 346-byte Siglent WAVEDESC block (FW 4.8.x little-endian layout)."""
    if len(desc) < 346:
        raise SDS824XError(f"Short WAVEDESC block: {len(desc)} bytes < 346")
    tdiv_idx = struct.unpack_from("<h", desc, 0x144)[0]
    probe = struct.unpack_from("<f", desc, 0x148)[0]
    point_num = struct.unpack_from("<i", desc, 0x74)[0]
    vdiv = struct.unpack_from("<f", desc, 0x9C)[0] * probe
    offset = struct.unpack_from("<f", desc, 0xA0)[0] * probe
    code_per_div = struct.unpack_from("<f", desc, 0xA4)[0]
    adc_bit = struct.unpack_from("<h", desc, 0xAC)[0]
    interval = struct.unpack_from("<f", desc, 0xB0)[0]
    delay = struct.unpack_from("<d", desc, 0xB4)[0]
    tdiv = _TDIV_ENUM[tdiv_idx] if 0 <= tdiv_idx < len(_TDIV_ENUM) else None
    return {
        "point_num": int(point_num),
        "vdiv": float(vdiv),
        "offset": float(offset),
        "code": float(code_per_div),
        "adc_bit": int(adc_bit),
        "interval": float(interval),
        "delay": float(delay),
        "tdiv": tdiv,
        "probe": float(probe),
    }


class SDS824XClient:
    """Safe SCPI client for SDS824X HD keeping acquisition out of Arm hang states."""

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        *,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        cmd_sleep_s: float = CMD_SLEEP_S,
        waveform_sleep_s: float = WAVEFORM_SLEEP_S,
        settle_scale: float = 1.0,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.cmd_sleep_s = cmd_sleep_s
        self.waveform_sleep_s = waveform_sleep_s
        self.settle_scale = settle_scale
        self._socket_factory = socket_factory
        self._sock: socket.socket | None = None

    def __enter__(self) -> "SDS824XClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        try:
            if self._sock is not None:
                try:
                    self.setup_channel(1, display=True, coupling="AC", scale=0.05, offset=0.0, attn=1, bwl=True, settle_s=0.1)
                    self.setup_timebase(2e-3, delay_s=0.0, settle_s=0.05)
                    self.run_auto()
                except (OSError, SDS824XError):
                    pass
        finally:
            self.close()
        return False

    def connect(self) -> None:
        if self._sock is not None:
            return
        sock = self._socket_factory(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.timeout_s)
        try:
            sock.connect((self.host, self.port))
        except OSError as exc:
            sock.close()
            raise ConnectionError(f"Cannot connect to SDS824X HD at {self.host}:{self.port}: {exc}") from exc
        self._sock = sock

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _require_sock(self) -> socket.socket:
        if self._sock is None:
            raise SDS824XError("SDS824XClient is not connected")
        return self._sock

    def send(self, command: str) -> None:
        sock = self._require_sock()
        try:
            sock.sendall((command.strip() + "\n").encode("ascii"))
        except OSError as exc:
            raise SDS824XError(f"SCPI send failed for {command!r}: {exc}") from exc
        if self.cmd_sleep_s > 0:
            time.sleep(self.cmd_sleep_s)

    def query(self, command: str) -> str:
        sock = self._require_sock()
        try:
            sock.sendall((command.strip() + "\n").encode("ascii"))
        except OSError as exc:
            raise SDS824XError(f"SCPI send failed for {command!r}: {exc}") from exc
        if self.cmd_sleep_s > 0:
            time.sleep(self.cmd_sleep_s)

        chunks: list[bytes] = []
        try:
            while True:
                chunk = sock.recv(65536)
                if not chunk:
                    break
                chunks.append(chunk)
                if b"\n" in chunk:
                    break
        except socket.timeout as exc:
            raise SDS824XError(f"SCPI timeout waiting for reply to {command!r}") from exc
        except OSError as exc:
            raise SDS824XError(f"SCPI recv failed for {command!r}: {exc}") from exc

        raw = b"".join(chunks).decode("ascii", errors="replace")
        if "\n" not in raw:
            raise SDS824XError(f"Incomplete SCPI reply to {command!r}: {raw!r}")
        return raw.strip()

    def _recv_exact(self, sock: socket.socket, nbytes: int, context: str) -> bytes:
        buf = bytearray()
        while len(buf) < nbytes:
            try:
                chunk = sock.recv(min(65536, nbytes - len(buf)))
            except socket.timeout as exc:
                raise SDS824XError(f"Timeout receiving {nbytes} bytes for {context}") from exc
            except OSError as exc:
                raise SDS824XError(f"Socket error receiving {context}: {exc}") from exc
            if not chunk:
                raise SDS824XError(f"EOF after {len(buf)}/{nbytes} bytes for {context}")
            buf.extend(chunk)
        return bytes(buf)

    def query_binary(self, command: str) -> bytes:
        sock = self._require_sock()
        try:
            sock.sendall((command.strip() + "\n").encode("ascii"))
        except OSError as exc:
            raise SDS824XError(f"SCPI binary send failed for {command!r}: {exc}") from exc
        if self.waveform_sleep_s > 0:
            time.sleep(self.waveform_sleep_s)

        header = b""
        while True:
            b = self._recv_exact(sock, 1, f"{command} '#' header")
            if b == b"#":
                break
            header += b
            if len(header) > 64:
                raise SDS824XError(f"Expected binary block for {command!r}, got {header!r}")

        n_digits_raw = self._recv_exact(sock, 1, f"{command} digit count")
        try:
            n_digits = int(n_digits_raw.decode("ascii"))
        except ValueError as exc:
            raise SDS824XError(f"Invalid IEEE block digit count {n_digits_raw!r}") from exc

        len_bytes = self._recv_exact(sock, n_digits, f"{command} byte count")
        data_len = int(len_bytes.decode("ascii"))
        data = self._recv_exact(sock, data_len, f"{command} payload")
        # Siglent FW 4.8.x may emit one or two trailing LF/CR bytes after DATA?.
        prev_timeout = sock.gettimeout()
        try:
            sock.settimeout(0.05)
            while True:
                extra = sock.recv(64)
                if not extra or set(extra) - {10, 13}:
                    break
        except OSError:
            pass
        finally:
            sock.settimeout(prev_timeout)
        return data

    def identify(self) -> str:
        ident = self.query("*IDN?")
        if "SDS824X" not in ident.upper():
            raise SDS824XError(f"Unexpected oscilloscope identity: {ident!r}")
        return ident

    def trigger_status(self) -> str:
        return self.query(":TRIGger:STATus?").strip()

    def wait_trigger_status(self, allowed: tuple[str, ...], *, timeout_s: float = 5.0, poll_s: float = 0.05) -> str:
        deadline = time.monotonic() + timeout_s
        last = ""
        sleep_s = poll_s * self.settle_scale
        while time.monotonic() <= deadline:
            last = self.trigger_status()
            if last in allowed:
                return last
            if sleep_s > 0:
                time.sleep(sleep_s)
        raise SDS824XError(f"Timeout waiting for trigger status in {allowed}, last={last!r}")

    def run_auto(self) -> None:
        self.send("TRMD AUTO")
        self.send(":TRIGger:RUN")

    def arm_single(self) -> None:
        self.send("TRMD SINGLE")
        self.send(":TRIGger:RUN")

    def stop(self) -> None:
        self.send(":STOP")

    def set_mdepth(self, depth: str = "100K") -> None:
        self.send(f":ACQuire:MDEPth {depth}")

    def get_sample_rate(self) -> float:
        return float(self.query(":ACQuire:SRATe?"))

    def setup_channel(
        self,
        channel: int,
        *,
        display: bool = True,
        coupling: str = "AC",
        scale: float | None = None,
        offset: float | None = None,
        attn: int | float | None = 1,
        bwl: bool | None = True,
        allow_gnd: bool = False,
        settle_s: float = 0.25,
    ) -> None:
        cpl_key = coupling.strip().upper()
        if cpl_key not in _COUPLING_MAP:
            raise ValueError(f"Unsupported coupling: {coupling!r}")
        cpl_code = _COUPLING_MAP[cpl_key]
        if cpl_code == "GND" and not allow_gnd:
            # Ground coupling can stall SDS824X HD FW 4.8.x trigger state if switched repeatedly.
            raise ValueError("GND coupling is disabled by default on SDS824X HD; pass allow_gnd=True if intentional")

        self.send(f"C{channel}:TRA {'ON' if display else 'OFF'}")
        if attn is not None:
            self.send(f"C{channel}:ATTN {attn}")
        self.send(f"C{channel}:CPL {cpl_code}")
        if bwl is not None:
            self.send(f"BWL C{channel},{'ON' if bwl else 'OFF'}")
        if scale is not None:
            self.send(f"C{channel}:VDIV {float(scale):.6E}")
        if offset is not None:
            self.send(f"C{channel}:OFST {float(offset):.6E}")
        wait_s = settle_s * self.settle_scale
        if wait_s > 0:
            time.sleep(wait_s)

    def setup_timebase(self, scale_s: float, *, delay_s: float = 0.0, settle_s: float = 0.20) -> None:
        self.send(f"TDIV {float(scale_s):.6E}")
        self.send(f"TRDL {float(delay_s):.6E}")
        wait_s = settle_s * self.settle_scale
        if wait_s > 0:
            time.sleep(wait_s)

    def setup_edge_trigger(
        self,
        *,
        source: str = "C1",
        slope: str = "RISing",
        level: float = 0.0,
        coupling: str = "DC",
    ) -> None:
        self.send(":TRIGger:TYPE EDGE")
        self.send(f":TRIGger:EDGE:SOURce {source}")
        self.send(f":TRIGger:EDGE:SLOPe {slope}")
        self.send(f":TRIGger:EDGE:COUPling {coupling}")
        self.send(f":TRIGger:EDGE:LEVel {float(level):.6E}")

    def measure(self, item: str, channel: int = 1) -> float:
        resp = self.query(f"C{channel}:PAVA? {item}")
        if "," not in resp:
            raise SDS824XError(f"Malformed PAVA reply for C{channel}:{item}: {resp!r}")
        tail = resp.split(",")[-1]
        if "****" in tail:
            return float("nan")
        num = "".join(ch for ch in tail if ch.isdigit() or ch in "+-.eE")
        if not num:
            raise SDS824XError(f"Unparseable PAVA numeric tail in {resp!r}")
        return float(num)

    def read_stopped_waveform(self, channel: int = 1) -> tuple[np.ndarray, np.ndarray, dict[str, float | int | None]]:
        self.send(f":WAVeform:SOURce C{channel}")
        self.send(":WAVeform:STARt 0")
        self.send(":WAVeform:WIDTh WORD")
        pre = self.query_binary(":WAVeform:PREamble?")
        meta = parse_wavedesc(pre)
        npts = int(meta["point_num"])
        code = float(meta["code"])
        vdiv = float(meta["vdiv"])
        offset = float(meta["offset"])
        interval = float(meta["interval"])
        if npts <= 0 or code <= 0.0 or vdiv <= 0.0 or interval <= 0.0:
            raise SDS824XError(f"Invalid waveform metadata while stopped: {meta}")

        self.send(f":WAVeform:POINt {npts}")
        raw = self.query_binary(":WAVeform:DATA?")
        expected_bytes = npts * 2
        if len(raw) != expected_bytes:
            raise SDS824XError(f"Waveform byte count mismatch: got {len(raw)}, expected {expected_bytes}")

        codes = np.frombuffer(raw, dtype="<i2").astype(np.float64)
        volts = (codes / code) * vdiv - offset
        tdiv = float(meta["tdiv"]) if meta["tdiv"] is not None else (npts * interval / HORI_DIVISIONS)
        t0 = -(tdiv * HORI_DIVISIONS / 2.0) + float(meta["delay"] or 0.0)
        times = t0 + np.arange(npts, dtype=np.float64) * interval
        return times, volts, meta

    def capture_auto_waveform(
        self,
        channel: int = 1,
        *,
        settle_s: float = 0.45,
        stop_wait_s: float = 0.20,
    ) -> tuple[np.ndarray, np.ndarray, dict[str, float | int | None]]:
        """Capture one complete frame under TRMD AUTO (accepting Auto/Trig'd/Arm free-run)."""
        self.run_auto()
        w_settle = settle_s * self.settle_scale
        if w_settle > 0:
            time.sleep(w_settle)
        self.wait_trigger_status(("Auto", "Trig'd", "Arm", "Stop"), timeout_s=5.0)
        self.stop()
        w_stop = stop_wait_s * self.settle_scale
        if w_stop > 0:
            time.sleep(w_stop)
        self.wait_trigger_status(("Stop",), timeout_s=3.0)
        try:
            return self.read_stopped_waveform(channel)
        finally:
            self.run_auto()
