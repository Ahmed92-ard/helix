"""
esp_comms.py — ESP-to-ESP Communication Handler

Manages WiFi TCP communication between ESP1 (ARM) and ESP2 (PRESS).

ESP1 initiates connection to ESP2 (press machine) and sends:
  - READY_PRESS  → ask press to execute press cycle
  - Waits for PRESS_ACK
  - Waits for PRESS_DONE

This module runs in a separate thread so motion doesn't block on press response.
"""

import socket
import threading
import time
from typing import Callable, Optional

from config import ESP2_IP, ESP2_PORT, ESP2_CONNECT_TIMEOUT


class ESPCommunicationError(Exception):
    """Raised when ESP-to-ESP communication fails."""
    pass


class ESPComms:
    """Handles ESP1 ↔ ESP2 communication over WiFi TCP."""

    def __init__(self, esp2_host: str, esp2_port: int,
                 on_status_change: Optional[Callable[[str], None]] = None,
                 on_emergency_stop: Optional[Callable[[], None]] = None):
        """
        Initialize ESP communication handler.

        Args:
            esp2_host: IP address of ESP2 (press machine)
            esp2_port: TCP port of ESP2
            on_status_change: Callback function for status updates
            on_emergency_stop: Callback function when STOP command received from ESP2
        """
        self.host = esp2_host
        self.port = esp2_port
        self.on_status_change = on_status_change
        self.on_emergency_stop = on_emergency_stop
        self._sock = None
        self._file = None
        self._lock = threading.Lock()
        self._connected = False
        self._press_complete = threading.Event()
        self._press_ack = threading.Event()
        self._listener_thread = None
        self._listener_running = False

    def connect(self, timeout: float = ESP2_CONNECT_TIMEOUT) -> bool:
        """
        Establish connection to ESP2.

        Returns:
            True if connected successfully, False otherwise.
        """
        try:
            self._sock = socket.create_connection(
                (self.host, self.port),
                timeout=timeout
            )
            # Disable Nagle's algorithm for immediate packet sending
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._sock.settimeout(0.5)  # reduced socket read timeout
            self._file = self._sock.makefile('r', encoding='utf-8', errors='ignore')
            self._connected = True
            self._log("Connected to ESP2")

            # Send initial handshake
            self._send_raw("HELLO")
            response = self._recv_raw(timeout=2.0)
            if response == "READY":
                self._log("ESP2 is ready")
                # Start background listener for STOP commands
                self._listener_running = True
                self._listener_thread = threading.Thread(
                    target=self._continuous_listener_thread,
                    daemon=True
                )
                self._listener_thread.start()
                return True
            else:
                self._log(f"Unexpected ESP2 response: {response}")
                self.disconnect()
                return False

        except Exception as e:
            self._log(f"Connection failed: {e}")
            self._connected = False
            return False

    def disconnect(self):
        """Close connection to ESP2."""
        with self._lock:
            self._listener_running = False
            if self._file:
                try:
                    self._file.close()
                except Exception:
                    pass
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
            self._connected = False
            self._log("Disconnected from ESP2")

    @property
    def is_connected(self) -> bool:
        """Check if connected to ESP2."""
        return self._connected

    def _send_raw(self, msg: str) -> None:
        """Send message to ESP2 (internal, no locking)."""
        with self._lock:
            if not self._connected or not self._sock:
                raise ESPCommunicationError("Not connected to ESP2")
            try:
                self._sock.sendall((msg + "\n").encode('utf-8'))
            except Exception as e:
                self._connected = False
                raise ESPCommunicationError(f"Send failed: {e}")

    def _recv_raw(self, timeout: Optional[float] = None) -> str:
        """Receive message from ESP2 (internal, no locking)."""
        if not self._connected or not self._file:
            raise ESPCommunicationError("Not connected to ESP2")

        try:
            line = self._file.readline()
            if line == "":
                raise ESPCommunicationError("ESP2 closed connection")
            return line.strip()
        except socket.timeout:
            return ""
        except Exception as e:
            self._connected = False
            raise ESPCommunicationError(f"Receive failed: {e}")

    def request_press(self, wait_timeout_s: float = 10.0) -> bool:
        """
        Request press cycle from ESP2 and wait for completion.

        This is a blocking call that:
          1. Sends READY_PRESS to ESP2
          2. Waits for PRESS_ACK
          3. Waits for PRESS_DONE

        Args:
            wait_timeout_s: Maximum seconds to wait for press to complete

        Returns:
            True if press completed successfully, False on timeout or error.
        """
        try:
            if not self._connected:
                if not self.connect():
                    return False

            # Clear completion events
            self._press_ack.clear()
            self._press_complete.clear()

            # Send READY_PRESS command
            self._log("Requesting press...")
            self._send_raw("READY_PRESS")

            # Start receiver thread to listen for responses
            receiver = threading.Thread(
                target=self._press_receiver_thread,
                daemon=True
            )
            receiver.start()

            # Wait for ACK and DONE
            ack_ok = self._press_ack.wait(timeout=1.0)  # reduced timeout, ESP2 responds immediately now
            if not ack_ok:
                self._log("Did not receive PRESS_ACK from ESP2")
                return False

            self._log("Press cycle in progress...")
            done_ok = self._press_complete.wait(timeout=wait_timeout_s)
            if not done_ok:
                self._log(f"Press did not complete within {wait_timeout_s}s")
                return False

            self._log("Press completed successfully")
            return True

        except ESPCommunicationError as e:
            self._log(f"Press request failed: {e}")
            self._connected = False
            return False
        except Exception as e:
            self._log(f"Unexpected error during press: {e}")
            return False

    def _press_receiver_thread(self):
        """Thread that listens for PRESS_ACK and PRESS_DONE."""
        try:
            # Wait for PRESS_ACK - should be immediate now
            response = self._recv_raw(timeout=1.0)
            if response == "PRESS_ACK":
                self._press_ack.set()
                self._log("Received PRESS_ACK")
            else:
                self._log(f"Expected PRESS_ACK, got: {response}")
                return

            # Wait for PRESS_DONE - ESP2 executes immediately after ACK
            response = self._recv_raw(timeout=5.0)
            if response == "PRESS_DONE":
                self._press_complete.set()
                self._log("Received PRESS_DONE")
            else:
                self._log(f"Expected PRESS_DONE, got: {response}")

        except Exception as e:
            self._log(f"Receiver thread error: {e}")

    def _continuous_listener_thread(self):
        """Background thread that continuously listens for STOP commands from ESP2."""
        while self._listener_running and self._connected:
            try:
                response = self._recv_raw(timeout=0.5)
                if response == "STOP":
                    self._log("[EMERGENCY STOP] Received STOP from switch/ESP2 - stopping immediately!")
                    if self.on_emergency_stop:
                        self.on_emergency_stop()
                elif response and response != "":
                    self._log(f"Received unexpected message: {response}")
            except Exception:
                # Socket timeout or other error - just continue listening
                pass

    def _log(self, msg: str):
        """Log message via callback if provided."""
        prefix = "[ESP-COMMS]"
        full_msg = f"{prefix} {msg}"
        if self.on_status_change:
            self.on_status_change(full_msg)
        else:
            print(full_msg)


def create_esp_comms_handler(esp2_ip: Optional[str] = None,
                             esp2_port: Optional[int] = None,
                             log_callback: Optional[Callable[[str], None]] = None,
                             on_emergency_stop: Optional[Callable[[], None]] = None) -> ESPComms:
    """
    Factory function to create ESP communication handler.

    Args:
        esp2_ip: IP address of ESP2 (or use config default)
        esp2_port: TCP port of ESP2 (or use config default)
        log_callback: Optional callback for log messages
        on_emergency_stop: Optional callback for STOP command from ESP2

    Returns:
        ESPComms instance
    """
    host = esp2_ip or ESP2_IP
    port = esp2_port or ESP2_PORT
    return ESPComms(host, port, on_status_change=log_callback, on_emergency_stop=on_emergency_stop)
