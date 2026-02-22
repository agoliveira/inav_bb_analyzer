#!/usr/bin/env python3
"""
INAV MSP - MultiWii Serial Protocol v2 communication for INAV flight controllers.

Handles serial communication with INAV FCs for:
  - Flight controller identification (firmware, craft name, board)
  - Dataflash blackbox download with progress reporting
  - Dataflash erase
  - (Future) CLI command send/receive, diff all, parameter read/write

MSP v2 frame format:
  $X<  flag(u8)  cmd(u16 LE)  size(u16 LE)  payload  crc8
  $X>  flag(u8)  cmd(u16 LE)  size(u16 LE)  payload  crc8

Usage:
    from inav_msp import INAVDevice

    with INAVDevice("/dev/ttyACM0") as fc:
        info = fc.get_info()
        print(f"Connected: {info['craft_name']} running {info['firmware']}")
        fc.download_blackbox("./blackbox", erase_after=True)
"""

import glob
import os
import struct
import sys
import time

try:
    import serial
except ImportError:
    serial = None  # Checked in open()

VERSION = "1.0.0"

# ─── MSP Command IDs ─────────────────────────────────────────────────────────

MSP_API_VERSION         = 1
MSP_FC_VARIANT          = 2
MSP_FC_VERSION          = 3
MSP_BOARD_INFO          = 4
MSP_BUILD_INFO          = 5
MSP_NAME                = 10
MSP_DATAFLASH_SUMMARY   = 70
MSP_DATAFLASH_READ      = 71
MSP_DATAFLASH_ERASE     = 72

# ─── MSP v2 CRC-8 DVB-S2 ─────────────────────────────────────────────────────

def _build_crc8_table():
    """Build CRC-8 lookup table using DVB-S2 polynomial (0xD5)."""
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return bytes(table)

_CRC8_TABLE = _build_crc8_table()


def _crc8_dvb_s2(data):
    """Compute CRC-8 DVB-S2 over a bytes-like object."""
    crc = 0
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc


# ─── MSP v2 Frame Encoding/Decoding ──────────────────────────────────────────

def msp_v2_encode(cmd, payload=b""):
    """Encode an MSP v2 request frame (direction: to FC)."""
    flag = 0
    size = len(payload)
    body = struct.pack("<BHH", flag, cmd, size) + payload
    crc = _crc8_dvb_s2(body)
    return b"$X<" + body + bytes([crc])


def msp_v2_decode(raw):
    """Decode an MSP v2 response frame.

    Returns:
        (cmd, payload) on success
        None on error/invalid frame
    """
    if len(raw) < 9:  # minimum: $X> + flag(1) + cmd(2) + size(2) + crc(1)
        return None

    # Find frame start
    idx = raw.find(b"$X")
    if idx < 0:
        return None

    raw = raw[idx:]
    if len(raw) < 9:
        return None

    direction = raw[2:3]
    if direction == b"!":
        # Error response from FC
        return None

    flag = raw[3]
    cmd = struct.unpack_from("<H", raw, 4)[0]
    size = struct.unpack_from("<H", raw, 6)[0]

    if len(raw) < 8 + size + 1:
        return None

    payload = raw[8:8 + size]
    expected_crc = raw[8 + size]

    # CRC covers flag + cmd + size + payload
    body = raw[3:8 + size]
    actual_crc = _crc8_dvb_s2(body)

    if actual_crc != expected_crc:
        return None

    return (cmd, payload)


# ─── Serial Port Discovery ───────────────────────────────────────────────────

def find_serial_ports():
    """Find candidate serial ports for INAV flight controllers.

    Returns list of port paths, ordered by likelihood (ACM first, then USB).
    """
    candidates = []
    # USB CDC (STM32 DFU/VCP) - most common for modern FCs
    candidates.extend(sorted(glob.glob("/dev/ttyACM*")))
    # FTDI / CP2102 / CH340
    candidates.extend(sorted(glob.glob("/dev/ttyUSB*")))
    # macOS
    candidates.extend(sorted(glob.glob("/dev/cu.usbmodem*")))
    candidates.extend(sorted(glob.glob("/dev/cu.SLAB_USBtoUART*")))
    # Windows (won't glob but keeping for reference)
    # COM3, COM4, etc.
    return candidates


def auto_detect_fc(baudrate=115200, timeout=2.0):
    """Scan serial ports and return the first one that responds as INAV.

    Returns:
        (INAVDevice, info_dict) on success - device is open, caller must close
        (None, None) if no FC found
    """
    ports = find_serial_ports()
    if not ports:
        return None, None

    for port in ports:
        dev = None
        try:
            dev = INAVDevice(port, baudrate=baudrate, timeout=timeout)
            dev.open()
            info = dev.get_info()
            if info and info.get("fc_variant") == "INAV":
                return dev, info
            dev.close()
        except Exception:
            try:
                if dev:
                    dev.close()
            except Exception:
                pass
            continue

    return None, None


# ─── INAV Device ──────────────────────────────────────────────────────────────

class INAVDevice:
    """MSP v2 communication with an INAV flight controller."""

    def __init__(self, port, baudrate=115200, timeout=2.0):
        self.port_path = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser = None
        self._info = None
        self._rxbuf = b""  # Persistent receive buffer for pipelining

    def open(self):
        """Open serial connection."""
        try:
            import serial
        except ImportError:
            print("  ERROR: pyserial is required for device communication.")
            print("    Install with: pip install pyserial --break-system-packages")
            sys.exit(1)

        self._ser = serial.Serial(
            port=self.port_path,
            baudrate=self.baudrate,
            timeout=self.timeout,
            write_timeout=self.timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        # Flush any stale data
        time.sleep(0.1)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._rxbuf = b""

    def close(self):
        """Close serial connection."""
        if self._ser and self._ser.is_open:
            try:
                self._ser.close()
            except BaseException:
                pass
        self._ser = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()

    def _send(self, cmd, payload=b"", flush=True):
        """Send an MSP v2 request."""
        # Only flush stale data for non-pipelined requests
        if flush:
            if self._ser.in_waiting:
                self._ser.read(self._ser.in_waiting)
            self._rxbuf = b""
        frame = msp_v2_encode(cmd, payload)
        self._ser.write(frame)

    def _recv(self, expected_cmd=None, timeout=None):
        """Receive and decode an MSP v2 response.

        Uses a persistent receive buffer (_rxbuf) so that extra bytes
        read from serial are preserved across calls - essential for
        pipelined reads where multiple responses arrive back-to-back.
        Returns (cmd, payload) or None.
        """
        if timeout is None:
            timeout = self.timeout

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            # Read any available serial data into persistent buffer
            waiting = self._ser.in_waiting
            if waiting > 0:
                self._rxbuf += self._ser.read(waiting)
            elif len(self._rxbuf) == 0:
                # Nothing buffered and nothing waiting - brief sleep
                time.sleep(0.0005)
                continue

            # Try to decode a frame from the buffer
            search_start = 0
            while True:
                idx = self._rxbuf.find(b"$X", search_start)
                if idx < 0:
                    # No frame start found - discard obvious garbage
                    # but keep tail that might be start of a partial frame
                    if len(self._rxbuf) > 2:
                        self._rxbuf = self._rxbuf[-2:]
                    break

                # Need at least 9 bytes for header: $X> + flag(1) + cmd(2) + size(2) + crc(1)
                if len(self._rxbuf) - idx < 9:
                    break  # Incomplete header, wait for more data

                # Check direction byte
                direction = self._rxbuf[idx + 2:idx + 3]
                if direction == b"!":
                    # Error frame - skip it
                    search_start = idx + 1
                    continue

                # Parse size to check if full frame is available
                size = struct.unpack_from("<H", self._rxbuf, idx + 6)[0]
                frame_len = 8 + size + 1  # header(8) + payload + crc(1)

                if len(self._rxbuf) - idx < frame_len:
                    break  # Incomplete frame, wait for more data

                # Full frame available - try decode
                result = msp_v2_decode(self._rxbuf[idx:idx + frame_len])
                if result is not None:
                    cmd, payload = result
                    if expected_cmd is None or cmd == expected_cmd:
                        # Consume this frame from the buffer
                        self._rxbuf = self._rxbuf[idx + frame_len:]
                        return result

                # CRC mismatch or wrong cmd - skip this $X marker
                search_start = idx + 1

            # If buffer has partial data but no new bytes arrived from
            # serial, sleep briefly to avoid hot-spinning.  Without this,
            # the loop burns 100% CPU while waiting for the rest of a
            # frame, which starves USB servicing in heavy processes
            # (numpy/scipy loaded) and causes cascading slowdowns.
            if waiting == 0:
                time.sleep(0.0002)

        return None

    def _request(self, cmd, payload=b"", timeout=None):
        """Send a request and wait for the matching response.

        Returns payload bytes or None on timeout/error.
        """
        self._send(cmd, payload)
        result = self._recv(expected_cmd=cmd, timeout=timeout)
        if result:
            return result[1]
        return None

    # ── FC Identification ─────────────────────────────────────────────────

    def get_fc_variant(self):
        """Get FC variant string (e.g., 'INAV')."""
        payload = self._request(MSP_FC_VARIANT)
        if payload and len(payload) >= 4:
            return payload[:4].decode("ascii", errors="ignore")
        return None

    def get_fc_version(self):
        """Get FC firmware version as (major, minor, patch) tuple."""
        payload = self._request(MSP_FC_VERSION)
        if payload and len(payload) >= 3:
            return (payload[0], payload[1], payload[2])
        return None

    def get_craft_name(self):
        """Get craft name string."""
        payload = self._request(MSP_NAME)
        if payload:
            return payload.decode("ascii", errors="ignore").strip("\x00").strip()
        return ""

    def get_board_info(self):
        """Get board identifier string."""
        payload = self._request(MSP_BOARD_INFO)
        if payload and len(payload) >= 4:
            return payload[:4].decode("ascii", errors="ignore")
        return None

    def get_info(self):
        """Get comprehensive FC identification.

        Returns dict with fc_variant, version, craft_name, board, firmware.
        """
        if self._info:
            return self._info

        variant = self.get_fc_variant()
        if not variant:
            return None

        version = self.get_fc_version()
        craft = self.get_craft_name()
        board = self.get_board_info()

        version_str = f"{version[0]}.{version[1]}.{version[2]}" if version else "?"
        firmware = f"{variant} {version_str}"

        self._info = {
            "fc_variant": variant,
            "version": version,
            "version_str": version_str,
            "craft_name": craft,
            "board": board,
            "firmware": firmware,
        }
        return self._info

    # ── Dataflash (Blackbox) ──────────────────────────────────────────────

    def get_dataflash_summary(self):
        """Get dataflash status and size info.

        Returns dict with:
            ready (bool): Flash ready for read
            supported (bool): Dataflash present (inferred from total_size > 0)
            sectors (int): Number of flash sectors
            total_size (int): Total flash size in bytes
            used_size (int): Used flash size in bytes
        """
        # Retry up to 3 times - first attempt after get_info() can
        # hit stale serial data on some FCs
        for attempt in range(3):
            payload = self._request(MSP_DATAFLASH_SUMMARY)
            if payload and len(payload) >= 13:
                break
            time.sleep(0.1)
        else:
            return None

        flags, sectors, total_size, used_size = struct.unpack_from("<BIII", payload, 0)

        # INAV uses flags byte as a simple ready boolean (0x01 = ready).
        # Unlike Betaflight, INAV does NOT set bit 1 for "supported".
        # Detect support from total_size > 0 instead.
        return {
            "ready": bool(flags & 0x01),
            "supported": total_size > 0,
            "sectors": sectors,
            "total_size": total_size,
            "used_size": used_size,
        }

    def read_dataflash_chunk(self, address, size=4096):
        """Read a chunk of dataflash at the given address.

        Args:
            address: Byte offset in dataflash
            size: Requested read size (FC may return less)

        Returns:
            (actual_address, data_bytes) or None on error
        """
        # MSP_DATAFLASH_READ request: address(u32) + requestedSize(u16)
        # Note: INAV does NOT require the compression flag byte
        req = struct.pack("<IH", address, size)
        payload = self._request(MSP_DATAFLASH_READ, req, timeout=5.0)

        if not payload or len(payload) < 5:
            return None

        resp_addr = struct.unpack_from("<I", payload, 0)[0]

        # INAV response format: address(u32) + data
        # (Betaflight v2 adds dataSize + compressedSize, but INAV doesn't)
        data = payload[4:]

        if len(data) == 0:
            return None

        return (resp_addr, data)

    def _send_dataflash_read(self, address, size=4096):
        """Send a dataflash read request WITHOUT waiting for response.

        Used for pipelining - fire multiple requests, collect responses later.
        Returns True if sent, False on write failure (buffer full).
        """
        req = struct.pack("<IH", address, size)
        frame = msp_v2_encode(MSP_DATAFLASH_READ, req)
        try:
            self._ser.write(frame)
            return True
        except serial.SerialTimeoutException:
            return False

    def _recv_dataflash_chunk(self, timeout=5.0):
        """Receive a single dataflash read response.

        Returns (address, data_bytes) or None on timeout.
        """
        result = self._recv(expected_cmd=MSP_DATAFLASH_READ, timeout=timeout)
        if result is None:
            return None
        payload = result[1]
        if not payload or len(payload) < 5:
            return None
        resp_addr = struct.unpack_from("<I", payload, 0)[0]
        data = payload[4:]
        if len(data) == 0:
            return None
        return (resp_addr, data)

    def download_blackbox(self, output_dir="./blackbox", erase_after=False,
                          progress_callback=None):
        """Download entire blackbox log from dataflash.

        Uses pipelined MSP reads for maximum throughput - multiple read
        requests are sent before collecting responses, eliminating idle
        time between round-trips.

        Args:
            output_dir: Directory to save the .bbl file
            erase_after: If True, erase dataflash after successful download
            progress_callback: Optional fn(bytes_read, total_bytes) for progress

        Returns:
            filepath of saved .bbl file, or None on failure
        """
        summary = self.get_dataflash_summary()
        if not summary:
            print("  ERROR: Could not read dataflash summary")
            return None

        if not summary["supported"]:
            print("  ERROR: Dataflash not supported on this board")
            return None

        if not summary["ready"]:
            print("  ERROR: Dataflash not ready")
            return None

        used = summary["used_size"]
        total = summary["total_size"]

        if used == 0:
            print("  No blackbox data on flash (0 bytes used)")
            return None

        # Get FC info for filename
        info = self.get_info()
        craft = info.get("craft_name", "unknown") if info else "unknown"
        # Sanitize craft name for filename
        safe_craft = "".join(c if c.isalnum() or c in "-_ " else "_" for c in craft)
        safe_craft = safe_craft.strip().replace(" ", "_")
        if not safe_craft:
            safe_craft = "blackbox"

        timestamp = time.strftime("%Y-%m-%d_%H%M%S")
        filename = f"{safe_craft}_{timestamp}.bbl"

        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, filename)

        # ── Determine optimal chunk size ──
        # Start with a single request to probe actual response size.
        # The FC returns MIN(requested, buffer_space, remaining_data).
        # Typical: 2048 on F4, 4096 on F7/H7 targets.
        probe = self.read_dataflash_chunk(0, 4096)
        if probe is None:
            # Fall back to smaller chunk
            probe = self.read_dataflash_chunk(0, 1024)
            if probe is None:
                print("  ERROR: Cannot read dataflash")
                return None

        chunk_size = len(probe[1])
        # Pipeline depth: how many requests to keep in-flight.
        # USB VCP has limited buffer - too many in-flight fills the FC's
        # USB output buffer and causes write timeouts. 4 is safe for F4/F7/H7.
        pipeline_depth = 4 if chunk_size >= 1024 else 1

        data_buf = bytearray()
        data_buf.extend(probe[1])  # Already have first chunk
        address = len(probe[1])

        start_time = time.monotonic()
        retries = 0
        max_retries = 10

        print(f"  Downloading {used / 1024:.0f}KB ({chunk_size}B chunks, "
              f"pipeline={pipeline_depth})...")

        # ── Pipelined download loop ──
        # Strategy: keep N requests in flight at all times.
        # Fire N requests, then enter a loop: receive 1 response, fire 1 request.
        # This keeps the FC busy reading flash while we process the previous chunk.

        if pipeline_depth > 1:
            # Flush any stale data
            if self._ser.in_waiting:
                self._ser.read(self._ser.in_waiting)
            self._rxbuf = b""

            in_flight = 0
            next_send_addr = address

            # Prime the pipeline gradually to avoid overwhelming USB buffer
            while in_flight < pipeline_depth and next_send_addr < used:
                if not self._send_dataflash_read(next_send_addr, chunk_size):
                    # Write failed - drain some responses first
                    time.sleep(0.01)
                    if self._ser.in_waiting:
                        self._rxbuf += self._ser.read(self._ser.in_waiting)
                    if not self._send_dataflash_read(next_send_addr, chunk_size):
                        break  # Still failing, proceed with what we have
                next_send_addr += chunk_size
                in_flight += 1
                # Small delay between initial sends to let FC start processing
                if in_flight < pipeline_depth:
                    time.sleep(0.002)

            drain_retries = 0
            max_drain_retries = 5
            while in_flight > 0 or next_send_addr < used:
                # Re-prime if pipeline drained but work remains
                if in_flight == 0 and next_send_addr < used:
                    # Flush any stale data in buffers before re-priming
                    if self._ser.in_waiting:
                        self._ser.read(self._ser.in_waiting)
                    self._rxbuf = b""
                    time.sleep(0.05 * (drain_retries + 1))

                    while in_flight < pipeline_depth and next_send_addr < used:
                        if self._send_dataflash_read(next_send_addr, chunk_size):
                            next_send_addr += chunk_size
                            in_flight += 1
                            time.sleep(0.002)
                        else:
                            time.sleep(0.01)
                            break

                if in_flight == 0:
                    drain_retries += 1
                    if drain_retries > max_drain_retries:
                        print(f"\n  ERROR: Download stalled at {len(data_buf)/1024:.0f}KB "
                              f"({address * 100 // used}%) - FC stopped responding")
                        return None
                    continue  # retry re-prime

                # Receive one response
                result = self._recv_dataflash_chunk(timeout=5.0)

                if result is None:
                    retries += 1
                    if retries > max_retries:
                        print(f"\n  ERROR: Too many read errors at offset {address}")
                        return None
                    # Re-send all in-flight requests from current address
                    time.sleep(0.05 * retries)
                    if self._ser.in_waiting:
                        self._ser.read(self._ser.in_waiting)
                    self._rxbuf = b""
                    in_flight = 0
                    next_send_addr = address
                    while in_flight < pipeline_depth and next_send_addr < used:
                        if not self._send_dataflash_read(next_send_addr, chunk_size):
                            time.sleep(0.01)
                            if not self._send_dataflash_read(next_send_addr, chunk_size):
                                break
                        next_send_addr += chunk_size
                        in_flight += 1
                        time.sleep(0.002)
                    continue

                retries = 0
                drain_retries = 0
                resp_addr, data = result

                # Handle out-of-order or duplicate responses
                if resp_addr == address:
                    data_buf.extend(data)
                    address += len(data)
                    in_flight -= 1

                    # Send next request to keep pipeline full
                    if next_send_addr < used:
                        if self._send_dataflash_read(next_send_addr, chunk_size):
                            next_send_addr += chunk_size
                            in_flight += 1
                        else:
                            # Write failed - FC buffer full, just continue draining
                            pass
                else:
                    # Got unexpected address - drain pipeline, resync
                    in_flight -= 1
                    if resp_addr > address:
                        address = resp_addr
                        data_buf.extend(data)
                        address += len(data)

                # Progress
                elapsed = time.monotonic() - start_time
                speed = len(data_buf) / elapsed if elapsed > 0 else 0
                pct = min(100, address * 100 // used)
                bar_width = 20
                filled = bar_width * pct // 100
                bar = "█" * filled + "░" * (bar_width - filled)
                print(f"\r  Downloading: {pct:3d}% [{bar}] "
                      f"{len(data_buf) / 1024:.0f}KB / {used / 1024:.0f}KB  "
                      f"{speed / 1024:.0f}KB/s", end="", flush=True)

                if progress_callback:
                    progress_callback(len(data_buf), used)

        else:
            # Simple sequential download (fallback for tiny chunks)
            while address < used:
                chunk = self.read_dataflash_chunk(address, chunk_size)

                if chunk is None:
                    retries += 1
                    if retries > max_retries:
                        print(f"\n  ERROR: Too many read errors at offset {address}")
                        return None
                    time.sleep(0.05 * retries)
                    if self._ser.in_waiting:
                        self._ser.read(self._ser.in_waiting)
                    continue

                retries = 0
                resp_addr, data = chunk
                if resp_addr != address:
                    address = resp_addr
                data_buf.extend(data)
                address += len(data)

                elapsed = time.monotonic() - start_time
                speed = len(data_buf) / elapsed if elapsed > 0 else 0
                pct = min(100, address * 100 // used)
                bar_width = 20
                filled = bar_width * pct // 100
                bar = "█" * filled + "░" * (bar_width - filled)
                print(f"\r  Downloading: {pct:3d}% [{bar}] "
                      f"{len(data_buf) / 1024:.0f}KB / {used / 1024:.0f}KB  "
                      f"{speed / 1024:.0f}KB/s", end="", flush=True)

                if progress_callback:
                    progress_callback(len(data_buf), used)

        print()  # newline after progress bar

        elapsed = time.monotonic() - start_time
        avg_speed = len(data_buf) / elapsed if elapsed > 0 else 0

        # Completeness check - don't save partial downloads as success
        completeness = len(data_buf) / used if used > 0 else 1.0
        if completeness < 0.95:
            print(f"  ✖ Download incomplete: {len(data_buf)/1024:.0f}KB of {used/1024:.0f}KB "
                  f"({completeness*100:.0f}%)")
            print(f"    Try: unplug/replug USB and retry")
            return None

        # Write file
        with open(filepath, "wb") as f:
            f.write(data_buf)

        print(f"  ✓ Saved: {filepath} ({len(data_buf) / 1024:.0f}KB in {elapsed:.1f}s, "
              f"{avg_speed / 1024:.0f}KB/s)")

        # Erase if requested
        if erase_after:
            self.erase_dataflash()

        return filepath

    def erase_dataflash(self):
        """Erase all blackbox data from dataflash."""
        print("  Erasing dataflash...", end="", flush=True)
        self._send(MSP_DATAFLASH_ERASE)

        # Erase can take a while - poll until ready
        for _ in range(60):  # up to 30 seconds
            time.sleep(0.5)
            summary = self.get_dataflash_summary()
            if summary and summary["ready"] and summary["used_size"] == 0:
                print(" done")
                return True

        # Check one more time
        summary = self.get_dataflash_summary()
        if summary and summary["used_size"] == 0:
            print(" done")
            return True

        print(" timeout (flash may still be erasing)")
        return False

    # ── CLI Mode (for diff all) ───────────────────────────────────────────

    def cli_command(self, command, timeout=5.0):
        """Send a CLI command and capture the response.

        INAV enters CLI mode when it receives '#' character.
        Commands are sent as plain text, responses end with '# ' prompt.

        Args:
            command: CLI command string (e.g., 'diff all')
            timeout: Max seconds to wait for response

        Returns:
            Response string (without prompt), or None on error
        """
        ser = self._ser

        # Flush any pending data
        if ser.in_waiting:
            ser.read(ser.in_waiting)

        # Enter CLI mode by sending '#'
        ser.write(b"#")
        time.sleep(0.3)

        # Read and discard the CLI banner
        if ser.in_waiting:
            ser.read(ser.in_waiting)

        # Send the command
        ser.write((command + "\n").encode("ascii"))
        time.sleep(0.1)

        # Read response until we get '# ' prompt
        buf = b""
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting)
                # Look for the CLI prompt at the end
                # INAV CLI prompt is "\r\n# " at the end of output
                if buf.rstrip().endswith(b"#") or buf.endswith(b"# "):
                    break
            else:
                time.sleep(0.01)

        # Exit CLI mode
        ser.write(b"exit\n")
        time.sleep(0.3)
        # Flush the exit response
        if ser.in_waiting:
            ser.read(ser.in_waiting)

        if not buf:
            return None

        # Decode and clean up
        try:
            text = buf.decode("ascii", errors="replace")
        except Exception:
            return None

        # Remove the command echo and trailing prompt
        lines = text.splitlines()
        # Skip echo of our command and trailing prompt
        result_lines = []
        for line in lines:
            line = line.rstrip()
            if line == command:
                continue  # Skip command echo
            if line == "#" or line == "# ":
                continue  # Skip prompt
            result_lines.append(line)

        return "\n".join(result_lines).strip()

    def get_diff_all(self, timeout=10.0):
        """Pull the full 'diff all' configuration from the FC.

        Returns:
            Raw diff output string, or None on error
        """
        return self.cli_command("diff all", timeout=timeout)


# ─── CLI Entrypoint ──────────────────────────────────────────────────────────

def main():
    """Standalone usage: identify FC and download blackbox."""
    import argparse

    parser = argparse.ArgumentParser(
        description="INAV MSP - Download blackbox logs directly from flight controller")
    parser.add_argument("--device", "-d", default="auto",
                        help="Serial port (e.g., /dev/ttyACM0) or 'auto' to scan")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baud rate (default: 115200)")
    parser.add_argument("--output-dir", "-o", default="./blackbox",
                        help="Directory to save downloaded logs (default: ./blackbox)")
    parser.add_argument("--erase", action="store_true",
                        help="Erase dataflash after successful download")
    parser.add_argument("--info-only", action="store_true",
                        help="Only show FC info, don't download")
    args = parser.parse_args()

    print(f"\n  ▲ INAV MSP v{VERSION}")

    # Connect
    fc = None
    if args.device == "auto":
        print("  Scanning for INAV flight controller...")
        fc, info = auto_detect_fc(baudrate=args.baud)
        if not fc:
            print("  ERROR: No INAV flight controller found.")
            ports = find_serial_ports()
            if ports:
                print(f"    Ports found but none responded as INAV: {', '.join(ports)}")
                print("    Make sure the FC is powered and not in DFU mode.")
            else:
                print("    No serial ports detected. Is the FC connected via USB?")
            sys.exit(1)
        print(f"  Found: {fc.port_path}")
    else:
        port = args.device
        if not os.path.exists(port):
            print(f"  ERROR: Port not found: {port}")
            sys.exit(1)
        print(f"  Connecting: {port}")
        fc = INAVDevice(port, baudrate=args.baud)
        fc.open()
        info = fc.get_info()

    try:
        if not info:
            print("  ERROR: No response from FC. Check connection and baud rate.")
            sys.exit(1)

        print(f"\n  {'─' * 50}")
        print(f"  Firmware:   {info['firmware']}")
        print(f"  Craft:      {info['craft_name'] or '(not set)'}")
        print(f"  Board:      {info['board'] or '?'}")

        summary = fc.get_dataflash_summary()
        if summary:
            used_kb = summary['used_size'] / 1024
            total_kb = summary['total_size'] / 1024
            pct = summary['used_size'] * 100 // summary['total_size'] if summary['total_size'] > 0 else 0
            print(f"  Dataflash:  {used_kb:.0f}KB / {total_kb:.0f}KB ({pct}% used)")
            print(f"  {'─' * 50}")

            if args.info_only:
                return

            if summary['used_size'] == 0:
                print("\n  No blackbox data to download.")
                return

            print()
            filepath = fc.download_blackbox(
                output_dir=args.output_dir,
                erase_after=args.erase,
            )

            if filepath:
                print(f"\n  Ready to analyze:")
                print(f"    python3 inav_blackbox_analyzer.py {filepath}")
        else:
            print("  Dataflash:  not available")
            print(f"  {'─' * 50}")

    except KeyboardInterrupt:
        print("\n  Interrupted.")
        sys.exit(1)
    except Exception as e:
        print(f"  ERROR: {e}")
        sys.exit(1)
    finally:
        if fc:
            fc.close()


if __name__ == "__main__":
    main()
