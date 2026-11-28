"""Fail-safe TCP/SCPI client for RIGOL DG922 Pro RF generator (192.168.99.115:5025)."""

from __future__ import annotations

import atexit
import math
import signal
import socket
import threading
import time
from typing import Callable

DEFAULT_HOST = "192.168.99.115"
DEFAULT_PORT = 5025
DEFAULT_TIMEOUT_S = 5.0
CMD_SETTLE_S = 0.02
HARD_MAX_VPP = 10.0

_REGISTRY_LOCK = threading.Lock()
_ACTIVE_CLIENTS: set["DG922Client"] = set()
_HOOKS_INSTALLED = False


class SCPIError(RuntimeError):
    """Raised when DG922 Pro SCPI communication, readback check, or error queue fails."""


def dbm_to_vpp(dbm: float, load_ohms: float = 50.0) -> float:
    """Convert dBm into a 50-ohm load to peak-to-peak volts, enforcing Vpp <= 10.0 V."""
    if not math.isfinite(dbm) or not math.isfinite(load_ohms) or load_ohms <= 0:
        raise ValueError(f"Invalid dbm={dbm} or load_ohms={load_ohms}")
    watts = 10.0 ** ((dbm - 30.0) / 10.0)
    v_pk = math.sqrt(2.0 * load_ohms * watts)
    vpp = 2.0 * v_pk
    if vpp > HARD_MAX_VPP:
        raise ValueError(f"Requested {dbm} dBm ({vpp:.4f} Vpp) exceeds hard safety cap {HARD_MAX_VPP:.1f} Vpp")
    return vpp


def _shutdown_all_clients() -> None:
    with _REGISTRY_LOCK:
        clients = list(_ACTIVE_CLIENTS)
    for client in clients:
        try:
            client.emergency_output_off()
        except (OSError, SCPIError):
            # Cleanup handler must attempt every registered instrument.
            pass


def _signal_handler(signum: int, frame) -> None:
    _shutdown_all_clients()
    if signum == signal.SIGINT:
        raise KeyboardInterrupt("SIGINT received; DG922 Pro RF output forced OFF")
    raise SystemExit(128 + signum)


def _install_safety_hooks() -> None:
    global _HOOKS_INSTALLED
    with _REGISTRY_LOCK:
        if _HOOKS_INSTALLED:
            return
        atexit.register(_shutdown_all_clients)
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, _signal_handler)
            except ValueError:
                # Signal handlers can only be set in the main interpreter thread.
                pass
        _HOOKS_INSTALLED = True


class DG922Client:
    """Hardened SCPI socket client with 50-ohm/zero-offset readback and guaranteed RF-off."""

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        *,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        cmd_settle_s: float = CMD_SETTLE_S,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.cmd_settle_s = cmd_settle_s
        self._socket_factory = socket_factory
        self._sock: socket.socket | None = None
        _install_safety_hooks()

    def __enter__(self) -> "DG922Client":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        try:
            self.output_off(channel=1, verify=False)
        finally:
            self.close()
        return False

    def connect(self) -> None:
        sock = self._socket_factory(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.timeout_s)
        try:
            sock.connect((self.host, self.port))
        except OSError as exc:
            sock.close()
            raise ConnectionError(f"Cannot connect to DG922 Pro at {self.host}:{self.port}: {exc}") from exc
        self._sock = sock
        with _REGISTRY_LOCK:
            _ACTIVE_CLIENTS.add(self)

    def close(self) -> None:
        with _REGISTRY_LOCK:
            _ACTIVE_CLIENTS.discard(self)
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def _require_sock(self) -> socket.socket:
        if self._sock is None:
            raise SCPIError("DG922Client is not connected")
        return self._sock

    def send(self, command: str) -> None:
        sock = self._require_sock()
        sock.sendall((command.strip() + "\n").encode("ascii"))
        if self.cmd_settle_s > 0:
            time.sleep(self.cmd_settle_s)

    def query(self, command: str) -> str:
        sock = self._require_sock()
        self.send(command)
        chunks: list[bytes] = []
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            chunks.append(chunk)
            if b"\n" in chunk:
                break
        data = b"".join(chunks)
        if b"\n" not in data:
            raise TimeoutError(f"Incomplete SCPI response for '{command}': {data!r}")
        return data.decode("ascii", errors="replace").strip()

    def check_errors(self) -> None:
        """Query :SYSTem:ERRor? and fast-fail on any non-zero SCPI error."""
        resp = self.query(":SYSTem:ERRor?")
        code_str = resp.split(",", 1)[0].strip()
        try:
            code = int(code_str)
        except ValueError as exc:
            raise SCPIError(f"Unparseable :SYST:ERR? response: {resp!r}") from exc
        if code != 0:
            raise SCPIError(f"DG922 Pro SCPI error {code}: {resp}")

    def identify(self) -> str:
        return self.query("*IDN?")

    def configure_rf_dbm(self, freq_hz: float, power_dbm: float, *, channel: int = 1) -> float:
        """Program CH1 (50 ohm load, 0 V DC offset, sine freq/Vpp) and verify readbacks."""
        if not math.isfinite(freq_hz) or freq_hz <= 0:
            raise ValueError(f"Invalid freq_hz={freq_hz}")
        vpp = dbm_to_vpp(power_dbm, load_ohms=50.0)
        self.send(f":OUTPut{channel}:STATe OFF")
        self.send(f":OUTPut{channel}:LOAD 50")
        self.send(f":SOURce{channel}:APPLy:SINusoid {freq_hz:.6g},{vpp:.6g},0,0")
        self.send(f":SOURce{channel}:VOLTage:OFFSet 0")
        self.verify_safe_config(channel=channel, expected_freq_hz=freq_hz, expected_vpp=vpp)
        self.check_errors()
        return vpp

    def verify_safe_config(
        self,
        *,
        channel: int = 1,
        expected_freq_hz: float | None = None,
        expected_vpp: float | None = None,
    ) -> None:
        """Mandatory SCPI readback check before enabling RF output."""
        load_val = float(self.query(f":OUTPut{channel}:LOAD?"))
        if abs(load_val - 50.0) > 0.1:
            raise SCPIError(f"Unsafe CH{channel} load impedance readback: {load_val} ohm (expected 50)")
        offs_val = float(self.query(f":SOURce{channel}:VOLTage:OFFSet?"))
        if abs(offs_val) > 1e-4:
            raise SCPIError(f"Unsafe CH{channel} DC offset readback: {offs_val} V (expected 0 V)")
        vpp_val = float(self.query(f":SOURce{channel}:VOLTage?"))
        if not math.isfinite(vpp_val) or vpp_val <= 0 or vpp_val > HARD_MAX_VPP:
            raise SCPIError(f"Unsafe CH{channel} Vpp readback: {vpp_val} V")
        if expected_vpp is not None and abs(vpp_val - expected_vpp) > max(0.01, 0.02 * expected_vpp):
            raise SCPIError(f"CH{channel} Vpp mismatch: got {vpp_val}, expected {expected_vpp}")
        if expected_freq_hz is not None:
            freq_val = float(self.query(f":SOURce{channel}:FREQuency?"))
            if abs(freq_val - expected_freq_hz) > max(1.0, 1e-3 * expected_freq_hz):
                raise SCPIError(f"CH{channel} frequency mismatch: got {freq_val}, expected {expected_freq_hz}")

    def output_on(self, channel: int = 1) -> None:
        """Verify 50-ohm load, 0 V offset, and Vpp <= 10 V before enabling RF output."""
        self.verify_safe_config(channel=channel)
        self.send(f":OUTPut{channel}:STATe ON")
        state = self.query(f":OUTPut{channel}:STATe?").upper()
        if state not in ("1", "ON"):
            raise SCPIError(f"Failed to enable CH{channel} output; readback={state!r}")
        self.check_errors()

    def output_off(self, channel: int = 1, *, verify: bool = True) -> None:
        """Disable channel output."""
        if self._sock is None:
            return
        self.send(f":OUTPut{channel}:STATe OFF")
        if verify:
            state = self.query(f":OUTPut{channel}:STATe?").upper()
            if state not in ("0", "OFF"):
                raise SCPIError(f"Failed to disable CH{channel} output; readback={state!r}")

    def emergency_output_off(self) -> None:
        """Best-effort forced RF shutoff on process exit or signal."""
        if self._sock is not None:
            try:
                self._sock.sendall(b":OUTPut1:STATe OFF\n")
                return
            except OSError:
                pass
        sock = self._socket_factory(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        try:
            sock.connect((self.host, self.port))
            sock.sendall(b":OUTPut1:STATe OFF\n")
        finally:
            sock.close()
