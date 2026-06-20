#!/usr/bin/env python3
"""
CharretteController WiFi client – separate telemetry & sensor logs
===================================================================
Creates a folder like log_20260425_153022/ containing:
  - telemetry.csv   (CSV header + data lines)
  - sensor.csv      (raw sensor samples, "S:" lines)

Console shows only user‑facing responses (ACK, ERR, GET ALL).
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


# ── Data classification ──────────────────────────────────────────────────
def _is_sensor(line: str) -> bool:
    """True for raw sensor stream lines (S:...)."""
    return line.startswith("S:")


def _is_telemetry_csv(line: str) -> bool:
    """True for CSV header or data lines."""
    if not line:
        return False
    if line.startswith("timestamp_ms"):
        return True
    # CSV data lines start with a digit (the timestamp)
    if line[0].isdigit():
        return True
    return False


# ── Console printer ──────────────────────────────────────────────────────
def _print_received(line: str) -> None:
    sys.stdout.write(ERASE_LINE + _colour_line(line) + "\n")
    if _HAS_READLINE:
        buf = readline.get_line_buffer()
        if buf:
            sys.stdout.write(PROMPT + buf)
    sys.stdout.flush()


# ── Reader thread ────────────────────────────────────────────────────────
_stop_event = threading.Event()


def _reader(sock: socket.socket, tele_file, sen_file) -> None:
    buffer = b""
    sock.settimeout(0.1)

    while not _stop_event.is_set():
        try:
            data = sock.recv(4096)
            if not data:
                print(f"\n{RED}Connection closed by remote{RESET}", file=sys.stderr)
                break
            buffer += data
            buffer = buffer.removesuffix(b"\r\n")
            while b"\n" in buffer:
                line_bytes, buffer = buffer.split(b"\n", 1)
                line = line_bytes.decode("utf-8", errors="replace").rstrip("\r")

                # Log to appropriate file
                if _is_sensor(line) and sen_file:
                    stripped_line = line.split(":")[1]
                    sen_file.write(stripped_line + "\n")
                    sen_file.flush()
                elif _is_telemetry_csv(line) and tele_file:
                    tele_file.write(line + "\n")
                    tele_file.flush()

                # Only show non‑data lines on console
                if not (_is_sensor(line) or _is_telemetry_csv(line)):
                    _print_received(line.strip())

        except socket.timeout:
            continue
        except Exception as e:
            print(f"\n{RED}Reader error: {e}{RESET}", file=sys.stderr)
            break

    _stop_event.set()


# ── Main ──────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="CharretteController WiFi client")
    parser.add_argument(
        "--logdir", help="Log directory name (default: log_<timestamp>)"
    )
    args = parser.parse_args()

    # Create a timestamped folder for log files
    folder = args.logdir or datetime.now().strftime("logs/log_%Y%m%d_%H%M%S")
    os.makedirs(folder, exist_ok=True)  # exist_ok=True safe though unlikely to collide

    tele_path = os.path.join(folder, "telemetry.csv")
    sens_path = os.path.join(folder, "sensor.csv")

    tele_file = open(tele_path, "w", encoding="utf-8")
    sens_file = open(sens_path, "w", encoding="utf-8")
    print(f"Logging to folder: {folder}/")
    print(f"  Telemetry : {tele_path}")
    print(f"  Sensor    : {sens_path}")

    print(f"{CYAN}CharretteController WiFi client{RESET}")
    print(f"Connecting to {HOST}:{PORT} …")

    try:
        sock = socket.create_connection((HOST, PORT), timeout=CONNECT_TIMEOUT)
    except (socket.timeout, OSError) as exc:
        print(f"{RED}Could not connect: {exc}{RESET}")
        tele_file.close()
        sens_file.close()
        sys.exit(1)

    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(
        f"{GREEN}Connected.{RESET}  Commands: SET <key> <value>  |  GET ALL  |  quit\n"
    )

    reader_thread = threading.Thread(
        target=_reader, args=(sock, tele_file, sens_file), daemon=True
    )
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
        tele_file.close()
        sens_file.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
