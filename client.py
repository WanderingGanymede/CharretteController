#!/usr/bin/env python3
"""
CharretteController WiFi client
================================
Logs all telemetry to a timestamped CSV file while showing only
user‑important responses (ACK, ERR, GET ALL) on the console.

Usage
-----
    python3 client.py                     # logs to telemetry_<timestamp>.csv
    python3 client.py --log mydata.csv    # custom filename
"""

import argparse
import os
import socket
import sys
import threading
from datetime import datetime

try:
    import readline

    _HAS_READLINE = True
except ImportError:
    _HAS_READLINE = False

# ── Config ────────────────────────────────────────────────────────────────
HOST = "192.168.4.1"
PORT = 8888
CONNECT_TIMEOUT = 6
PROMPT = "> "


# ── ANSI helpers ──────────────────────────────────────────────────────────
def _supports_colour() -> bool:
    return hasattr(sys.stdout, "isatty") and sys.stdout.isatty()


_COL = _supports_colour()
RESET = "\033[0m" if _COL else ""
GREY = "\033[90m" if _COL else ""
GREEN = "\033[92m" if _COL else ""
YELLOW = "\033[93m" if _COL else ""
RED = "\033[91m" if _COL else ""
CYAN = "\033[96m" if _COL else ""

ERASE_LINE = "\r\033[K"


def _colour_line(line: str) -> str:
    s = line.strip()
    if s.startswith("ACK"):
        return GREEN + line + RESET
    if s.startswith("---"):
        return CYAN + line + RESET
    if s.startswith("ERR"):
        return RED + line + RESET
    if s.startswith("timestamp_ms"):
        return YELLOW + line + RESET
    return line


# ── Telemetry filter (console only) ───────────────────────────────────────
def _is_telemetry(line: str) -> bool:
    """Return True if the line is high‑rate telemetry (not printed)."""
    if not line:
        return False
    # Sensor streaming lines start with "S:"
    if line.startswith("S:"):
        return False
    # CSV header or data lines start with "timestamp_ms" or a digit
    if line[0].isdigit() or line.startswith("timestamp_ms"):
        return True
    return False


# ── Incoming‑line printer (non‑blocking) ──────────────────────────────────
def _print_received(line: str) -> None:
    sys.stdout.write(ERASE_LINE + _colour_line(line) + "\n")
    if _HAS_READLINE:
        buf = readline.get_line_buffer()
        if buf:
            sys.stdout.write(PROMPT + buf)
    sys.stdout.flush()


# ── Reader thread: drain socket as fast as possible ───────────────────────
_stop_event = threading.Event()


def _reader(sock: socket.socket, log_file) -> None:
    """Read raw data, display non‑telemetry, log only telemetry lines."""
    buffer = b""
    sock.settimeout(0.1)

    while not _stop_event.is_set():
        try:
            data = sock.recv(4096)
            if not data:
                print(f"\n{RED}Connection closed by remote{RESET}", file=sys.stderr)
                break
            buffer += data

            # Process complete lines
            while b"\n" in buffer:
                line_bytes, buffer = buffer.split(b"\n", 1)
                line = line_bytes.decode("utf-8", errors="replace").rstrip("\r")
                print("--------------")
                # Log telemetry lines to file
                if log_file and _is_telemetry(line):
                    log_file.write(line + "\n")
                    log_file.flush()

                # Display non‑telemetry lines (user responses)
                if not _is_telemetry(line):
                    _print_received(line)

        except socket.timeout:
            continue
        except Exception as e:
            print(f"\n{RED}Reader error: {e}{RESET}", file=sys.stderr)
            break

    _stop_event.set()


# ── Main ───────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="CharretteController WiFi client")
    parser.add_argument(
        "--log", help="Log file name (default: telemetry_<timestamp>.csv)"
    )
    args = parser.parse_args()

    # Generate default timestamped filename if not provided
    if args.log:
        log_filename = args.log
    else:
        log_filename = datetime.now().strftime("logs/telemetry_%Y%m%d_%H%M%S.csv")

    log_file = open(log_filename, "w", encoding="utf-8")
    print(f"Logging all telemetry to: {log_filename}")

    print(f"{CYAN}CharretteController WiFi client{RESET}")
    print(f"Connecting to {HOST}:{PORT} …")

    try:
        sock = socket.create_connection((HOST, PORT), timeout=CONNECT_TIMEOUT)
    except (socket.timeout, OSError) as exc:
        print(f"{RED}Could not connect: {exc}{RESET}")
        log_file.close()
        sys.exit(1)

    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(
        f"{GREEN}Connected.{RESET}  Commands: SET <key> <value>  |  GET ALL  |  quit\n"
    )

    reader_thread = threading.Thread(target=_reader, args=(sock, log_file), daemon=True)
    reader_thread.start()

    try:
        while not _stop_event.is_set():
            try:
                raw = input(PROMPT)
            except EOFError:
                break

            cmd = raw.strip()
            if not cmd:
                continue
            if cmd.lower() in ("quit", "exit", "q"):
                print("Disconnecting …")
                break

            try:
                sock.sendall((cmd + "\r\n").encode("utf-8"))
                sys.stdout.write(f"{GREY}  → {cmd}{RESET}\n")
                sys.stdout.flush()
            except OSError as exc:
                print(f"{RED}Send failed: {exc}{RESET}")
                break

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        _stop_event.set()
        sock.close()
        log_file.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
