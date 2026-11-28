"""Fail-safe TCP/SCPI client for RIGOL DM858E 5.5-digit bench multimeter (192.168.99.147:5025)."""

from __future__ import annotations

import math
import socket
import time
from typing import Callable

DEFAULT_HOST = "192.168.99.147"
DEFAULT_PORT = 5025
DEFAULT_TIMEOUT_S = 10.0
CONNECT_SETTLE_S = 0.10
CMD_SETTLE_S = 0.20
OVERLOAD_SENTINEL = 1e37


class DM858EError(RuntimeError):
    """Raised when DM858E SCPI communication, range check, or error queue fails."""


class DM858EClient:
    """Strict SCPI socket client for RIGOL DM858E with RIGOL LAN timing and fast-fail errors."""

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        *,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        connect_settle_s: float = CONNECT_SETTLE_S,
        cmd_settle_s: float = CMD_SETTLE_S,
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout_s = timeout_s
        self.connect_settle_s = connect_settle_s
        self.cmd_settle_s = cmd_settle_s
        self._socket_factory = socket_factory
        self._sock: socket.socket | None = None

    def __enter__(self) -> "DM858EClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        try:
            if self._sock is not None:
                try:
                    self.send("CONF:VOLT:DC AUTO")
                except (OSError, DM858EError):
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
            raise ConnectionError(
                f"Cannot connect to DM858E at {self.host}:{self.port}: {exc}"
            ) from exc
        self._sock = sock
        if self.connect_settle_s > 0:
            time.sleep(self.connect_settle_s)

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _require_sock(self) -> socket.socket:
        if self._sock is None:
            raise DM858EError("DM858EClient is not connected")
        return self._sock

    def send(self, command: str) -> None:
        sock = self._require_sock()
        payload = (command.strip() + "\n").encode("ascii")
        try:
            sock.sendall(payload)
        except OSError as exc:
            raise DM858EError(f"SCPI send failed for {command!r}: {exc}") from exc
        if self.cmd_settle_s > 0:
            time.sleep(self.cmd_settle_s)

    def query(self, command: str) -> str:
        sock = self._require_sock()
        payload = (command.strip() + "\n").encode("ascii")
        try:
            sock.sendall(payload)
        except OSError as exc:
            raise DM858EError(f"SCPI send failed for {command!r}: {exc}") from exc
        if self.cmd_settle_s > 0:
            time.sleep(self.cmd_settle_s)

        chunks: list[bytes] = []
        try:
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                chunks.append(chunk)
                if b"\n" in chunk:
                    break
        except socket.timeout as exc:
            raise DM858EError(f"SCPI timeout waiting for reply to {command!r}") from exc
        except OSError as exc:
            raise DM858EError(f"SCPI recv failed for {command!r}: {exc}") from exc

        raw = b"".join(chunks).decode("ascii", errors="replace")
        if "\n" not in raw:
            raise DM858EError(f"Incomplete SCPI reply to {command!r}: {raw!r}")
        reply = raw.strip()
        if not reply:
            raise DM858EError(f"Empty SCPI reply to {command!r}")
        return reply

    def query_float(self, command: str) -> float:
        reply = self.query(command)
        token = reply.split(",")[0].strip()
        try:
            val = float(token)
        except ValueError as exc:
            raise DM858EError(f"Non-numeric SCPI reply for {command!r}: {reply!r}") from exc
        if math.isnan(val):
            raise DM858EError(f"NaN SCPI reply for {command!r}: {reply!r}")
        if abs(val) >= OVERLOAD_SENTINEL:
            return float("inf")
        return val

    def identify(self) -> str:
        ident = self.query("*IDN?")
        if "DM858E" not in ident.upper():
            raise DM858EError(f"Unexpected instrument identity: {ident!r}")
        return ident

    def clear_status(self) -> None:
        self.send("*CLS")

    def check_errors(self) -> None:
        err = self.query(":SYSTem:ERRor?")
        code_str = err.split(",", 1)[0].strip()
        try:
            code = int(code_str)
        except ValueError as exc:
            raise DM858EError(f"Malformed :SYSTem:ERRor? reply: {err!r}") from exc
        if code != 0:
            raise DM858EError(f"DM858E SCPI error queue non-zero: {err}")

    def query_function(self) -> str:
        return self.query(":FUNCtion?")

    def query_range(self) -> float:
        return self.query_float(":SENSe:VOLTage:DC:RANGe?")

    def query_nplc(self) -> float:
        return self.query_float(":SENSe:VOLTage:DC:NPLC?")

    def configure_dc_voltage(
        self,
        vrange: str | float = "AUTO",
        nplc: float | None = None,
        *,
        verify_error: bool = True,
    ) -> tuple[float, float]:
        if isinstance(vrange, str) and vrange.strip().upper() == "AUTO":
            self.send("CONF:VOLT:DC AUTO")
        else:
            self.send(f"CONF:VOLT:DC {float(vrange):g}")
        if nplc is not None:
            self.send(f":SENSe:VOLTage:DC:NPLC {float(nplc):g}")
        if verify_error:
            self.check_errors()
        return self.query_range(), self.query_nplc()

    def read(self) -> float:
        return self.query_float("READ?")

    def read_finite(self) -> float:
        val = self.read()
        if not math.isfinite(val):
            raise DM858EError(f"Non-finite DM858E reading (overload?): {val}")
        return val

    def measure_dc_voltage(self) -> float:
        return self.query_float("MEAS:VOLT:DC?")
