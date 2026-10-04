"""
transport.py — Transport abstraction layer (USB Serial / WiFi TCP)

Both classes expose an identical interface:
    send(text)   → None
    readline()   → str   (empty on timeout, raises on disconnect)
    close()      → None
    is_open      → bool
    name         → str
"""

import socket
import threading

import serial

from config import (
    WIFI_SOCKET_TIMEOUT,
    WIFI_CONNECT_TIMEOUT,
)


# ─────────────────────────────────────────────────────────
#  USB SERIAL
# ─────────────────────────────────────────────────────────
class SerialTransport:
    """USB serial via pyserial."""
    name = "USB"

    def __init__(self, port: str, baud: int):
        self._ser = serial.Serial(port, baud, timeout=1)

    def send(self, text: str):
        self._ser.write((text + "\n").encode("utf-8"))

    def readline(self) -> str:
        """Returns stripped line, or '' on timeout. Raises OSError on disconnect."""
        raw = self._ser.readline()
        return raw.decode("utf-8", errors="ignore").strip()

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass

    @property
    def is_open(self) -> bool:
        return self._ser.is_open


# ─────────────────────────────────────────────────────────
#  WIFI TCP
# ─────────────────────────────────────────────────────────
class WifiTransport:
    """WiFi TCP socket.

    readline() raises ConnectionResetError when the remote end closes the
    connection (makefile().readline() returns '' on EOF, which we treat as
    a disconnect rather than silently swallowing it).

    TCP keepalive is enabled so the OS detects silent drops (router timeout,
    ESP32 reset) within ~15 seconds instead of hanging forever.
    """
    name = "WiFi"

    def __init__(self, host: str, port: int):
        self._sock = socket.create_connection((host, port), timeout=WIFI_CONNECT_TIMEOUT)
        self._sock.settimeout(WIFI_SOCKET_TIMEOUT)

        # ── TCP keepalive ─────────────────────────────────────────────
        # Keeps the connection alive and detects silent drops quickly.
        # Without this, a WiFi glitch silently hangs the readline() call.
        import platform
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if platform.system() == "Linux":
            # Start sending keepalives after 5 s of idle
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE,  5)
            # Re-send every 3 s
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
            # Drop after 4 missed keepalives (~12 s total)
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT,   4)

        self._file = self._sock.makefile("r", encoding="utf-8", errors="ignore")
        self._lock = threading.Lock()
        self._open = True

    def send(self, text: str):
        with self._lock:
            self._sock.sendall((text + "\n").encode("utf-8"))

    def readline(self) -> str:
        """Returns stripped line on data, '' on timeout, raises on disconnect.

        socket.makefile() on some Python versions raises
        OSError('cannot read from timed out object') instead of socket.timeout
        on a read timeout.  We catch both and treat them as normal empty-poll
        cycles.  A genuine EOF (remote close) is signalled by readline() == ''.
        """
        try:
            line = self._file.readline()
            if line == "":
                raise ConnectionResetError("ESP32 closed the connection (EOF)")
            return line.strip()
        except OSError as e:
            if "timed out" in str(e).lower():
                return ""
            raise

    def close(self):
        self._open = False
        try:
            self._sock.close()
        except Exception:
            pass

    @property
    def is_open(self) -> bool:
        return self._open
