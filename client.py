#!/usr/bin/env python3
"""
CharretteController WiFi client
================================
Connects to the PicoTrailer SoftAP and multiplexes telemetry output and
command input over a single TCP connection.

Incoming telemetry lines print above the prompt without disturbing whatever
you are currently typing — readline's buffer is preserved and redrawn each
time a new line arrives (only when you have actually started typing).

Usage
-----
1. Connect your laptop to the "PicoTrailer" Wi-Fi network.
2. Run:
       python3 client.py

Commands
--------
  GET ALL              — print all current tunable parameter values
  SET <key> <value>    — update a parameter live, e.g.  SET kp1 2.5

Tunable keys
------------
  kp1  ki1  kd1        PID gains, boost/ride mode  (K1)
  kp2  ki2  kd2        PID gains, walk mode        (K2)
  alpha                sensor threshold to engage PID
  beta0  beta1         decel threshold per mode
  gamma0 gamma1        brake threshold per mode
  sp0    sp1           PID setpoint per mode
  minA   maxA          motor current limits (A)
  minBrk maxBrk        brake current limits (A)
  rollThr              min speed considered "rolling" (km/h)
  brakeThr             brake thumb-throttle dead-zone

Press Ctrl+C or type 'quit' to disconnect.
"""

import socket
import sys
import threading

try:
    import readline

    _HAS_READLINE = True
except ImportError:
    _HAS_READLINE = False

# ── Config ────────────────────────────────────────────────────────────────────

HOST = "192.168.4.1"
PORT = 8888
CONNECT_TIMEOUT = 6  # seconds
PROMPT = "> "

# ── ANSI helpers ──────────────────────────────────────────────────────────────


def _supports_colour() -> bool:
    return hasattr(sys.stdout, "isatty") and sys.stdout.isatty()


_COL = _supports_colour()
RESET = "\033[0m" if _COL else ""
GREY = "\033[90m" if _COL else ""
GREEN = "\033[92m" if _COL else ""
YELLOW = "\033[93m" if _COL else ""
RED = "\033[91m" if _COL else ""
CYAN = "\033[96m" if _COL else ""

ERASE_LINE = "\r\033[K"  # move to col 0 + erase to end of line


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


# ── Incoming-line printer ─────────────────────────────────────────────────────


def _print_received(line: str) -> None:
    """
    Print one line received from the Pico without clobbering what the user
    is currently typing.

    Sequence:
      1. Erase the current terminal line  (wipes prompt + partial input)
      2. Print the received data + newline
      3. If the user has already started typing, reprint  '> <partial>'
         so it reappears below the new data.

    Step 3 is skipped when the buffer is empty (e.g. between commands) to
    avoid double-printing the prompt and confusing readline.
    """
    # Erase current line, print received data
    sys.stdout.write(ERASE_LINE + _colour_line(line) + "\n")

    # Only reprint the partial input when there is actually something typed.
    # Reprinting an empty prompt here would leave a stray "> " that readline
    # then duplicates when it redraws its own prompt.
    if _HAS_READLINE:
        buf = readline.get_line_buffer()
        if buf:
            sys.stdout.write(PROMPT + buf)

    sys.stdout.flush()


# ── Reader thread ─────────────────────────────────────────────────────────────

_stop_event = threading.Event()


def _reader(sock: socket.socket) -> None:
    """Background thread: read lines from the socket and display them."""
    try:
        f = sock.makefile("r", encoding="utf-8", errors="replace")
        for line in f:
            # Skip high-frequency sensor stream lines (S:...) — those are
            # intended for graph.py, not the command console.
            # if line.startswith("S:"):
            #    continue
            _print_received(line.rstrip())
    except Exception:
        pass
    finally:
        _stop_event.set()


# ── Main ───────────────────────────────────────────────────────────────────────


def main() -> None:
    print(f"{CYAN}CharretteController WiFi client{RESET}")
    print(f"Connecting to {HOST}:{PORT} …")

    try:
        sock = socket.create_connection((HOST, PORT), timeout=CONNECT_TIMEOUT)
    except socket.timeout:
        print(f"{RED}Timed out — is the Pico running and the hotspot visible?{RESET}")
        sys.exit(1)
    except OSError as exc:
        print(f"{RED}Could not connect: {exc}{RESET}")
        sys.exit(1)

    # Disable Nagle so short command strings go out immediately
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    print(
        f"{GREEN}Connected.{RESET}  Commands: SET <key> <value>  |  GET ALL  |  quit\n"
    )

    reader_thread = threading.Thread(target=_reader, args=(sock,), daemon=True)
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
                sock.sendall((cmd + "\n").encode("utf-8"))
                # Subtle grey echo so the user knows the send happened.
                # The real response (ACK / PARAM lines) arrives as a received line.
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
        print("Disconnected.")


if __name__ == "__main__":
    main()
