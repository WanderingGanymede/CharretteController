#!/usr/bin/env python3
"""
graph.py — Real-time weight sensor plot for CharretteController
===============================================================
Connects to the PicoTrailer SoftAP and plots raw HX711 sensor samples
so you can characterise noise and evaluate filter settings.

Two lines:
  Blue (thin) : valeurCapteurInstant — raw sample each time HX711 is ready
  Red (thick) : valeurCapteurMoyenne — Pico's own moving-average filter

Stats panel shows std-dev, peak-to-peak and sample rate over the
visible window so you can quantify the noise floor at a glance.

Usage
-----
1. Pico must have  wifiStreamSensor = true  (the default).
2. Connect laptop to the "PicoTrailer" Wi-Fi network.
3. nix-shell   (picks up pyqt5 for the interactive backend)
4. python3 graph.py

The Pico sends batched lines every 300 ms in the format:
    S:<millis>,<instant_kg>,<filtered_kg>
"""

# ── Backend must be set BEFORE importing pyplot ───────────────────────────────
import matplotlib

# WebAgg serves the plot as a local webpage — works on Wayland, X11, anything
# with a browser. No extra packages needed beyond matplotlib itself.
matplotlib.use("WebAgg")
matplotlib.rcParams["webagg.port"] = 8050  # avoid clash with Pico's 8888
matplotlib.rcParams["webagg.open_in_browser"] = False  # we'll print the URL
print(f"[graph] matplotlib backend: {matplotlib.get_backend()}")
print("[graph] plot will be served at  http://127.0.0.1:8050  — open in browser")

import collections
import socket
import sys
import threading

import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────

HOST = "192.168.4.1"
PORT = 8888
CONNECT_TIMEOUT = 6  # seconds
WINDOW_SECONDS = 15  # seconds of history visible in the plot
MAX_SAMPLES = 3000  # hard cap on deque length

# ── Shared state (reader thread writes, animation callback reads) ─────────────

_lock = threading.Lock()
_times = collections.deque(maxlen=MAX_SAMPLES)  # seconds, normalised to 0
_instant = collections.deque(maxlen=MAX_SAMPLES)  # kg
_filtered = collections.deque(maxlen=MAX_SAMPLES)  # kg
_sample_count = 0
_t0 = None  # timestamp (s) of first sample — set once

_stop_event = threading.Event()


# ── Socket reader ─────────────────────────────────────────────────────────────


def _reader(sock: socket.socket) -> None:
    """
    Background thread.  Reads lines from the socket and parses S: lines.
    Every non-S: line is printed so you can see what else the Pico sends.
    Exceptions are printed rather than swallowed so bugs are visible.
    """
    global _sample_count, _t0
    try:
        f = sock.makefile("r", encoding="utf-8", errors="replace")
        for raw_line in f:
            line = raw_line.strip()

            if not line:
                continue

            # ── Non-sensor lines: print for visibility ────────────────────
            if not line.startswith("S:"):
                print(f"[graph] non-S line: {line}")
                continue

            # ── Parse  S:<millis>,<instant>,<filtered> ────────────────────
            try:
                parts = line[2:].split(",")
                millis_ms = int(parts[0])
                v_instant = float(parts[1])
                v_filtered = float(parts[2])
            except (ValueError, IndexError) as exc:
                print(f"[graph] parse error on '{line}': {exc}")
                continue

            t_sec = millis_ms / 1000.0

            with _lock:
                if _t0 is None:
                    _t0 = t_sec
                    print("[graph] first sample received — plot will update shortly")
                _times.append(t_sec - _t0)
                _instant.append(v_instant)
                _filtered.append(v_filtered)
                _sample_count += 1

            # Progress heartbeat every 50 samples
            if _sample_count % 50 == 0:
                print(f"[graph] {_sample_count} samples received")

    except Exception as exc:
        print(f"[graph] reader error: {exc}")
    finally:
        print("[graph] reader thread exiting — connection lost?")
        _stop_event.set()


# ── Figure construction ───────────────────────────────────────────────────────


def _build_figure():
    fig = plt.figure(figsize=(13, 6))
    fig.patch.set_facecolor("#1e1e1e")

    gs = gridspec.GridSpec(
        2,
        2,
        figure=fig,
        width_ratios=[4, 1],
        height_ratios=[3, 1],
        hspace=0.08,
        wspace=0.28,
    )

    # ── Main waveform ─────────────────────────────────────────────────────
    ax_main = fig.add_subplot(gs[0, 0])
    ax_main.set_facecolor("#2d2d2d")
    ax_main.tick_params(colors="white", labelbottom=False)
    ax_main.set_ylabel("Weight (kg)", color="white")
    ax_main.set_title("Weight sensor — raw vs filtered", color="white")
    for sp in ax_main.spines.values():
        sp.set_edgecolor("#555555")
    ax_main.grid(True, color="#444444", linewidth=0.5, linestyle="--")

    (line_instant,) = ax_main.plot(
        [], [], color="#5599ff", linewidth=0.8, alpha=0.75, label="Instant (raw)"
    )
    (line_filtered,) = ax_main.plot(
        [], [], color="#ff5555", linewidth=1.8, label="Filtered (Pico avg)"
    )
    ax_main.legend(
        loc="upper left",
        facecolor="#333333",
        edgecolor="#555555",
        labelcolor="white",
    )

    # ── Residual (instant − filtered) ────────────────────────────────────
    ax_resid = fig.add_subplot(gs[1, 0], sharex=ax_main)
    ax_resid.set_facecolor("#2d2d2d")
    ax_resid.tick_params(colors="white")
    ax_resid.set_xlabel("Time (s)", color="white")
    ax_resid.set_ylabel("Residual (kg)", color="white", fontsize=8)
    for sp in ax_resid.spines.values():
        sp.set_edgecolor("#555555")
    ax_resid.grid(True, color="#444444", linewidth=0.5, linestyle="--")
    ax_resid.axhline(0, color="#888888", linewidth=0.8, linestyle=":")

    (line_resid,) = ax_resid.plot([], [], color="#aaddaa", linewidth=0.7, alpha=0.8)

    # ── Stats panel ───────────────────────────────────────────────────────
    ax_stats = fig.add_subplot(gs[:, 1])
    ax_stats.set_facecolor("#2d2d2d")
    ax_stats.axis("off")
    stats_text = ax_stats.text(
        0.05,
        0.95,
        "Waiting for data…",
        transform=ax_stats.transAxes,
        fontsize=9,
        verticalalignment="top",
        fontfamily="monospace",
        color="white",
    )

    return fig, ax_main, ax_resid, line_instant, line_filtered, line_resid, stats_text


# ── Animation update ──────────────────────────────────────────────────────────


def _make_updater(
    ax_main, ax_resid, line_instant, line_filtered, line_resid, stats_text
):

    def update(_frame):
        with _lock:
            if len(_times) < 2:
                return line_instant, line_filtered, line_resid, stats_text

            t_arr = np.array(_times)
            i_arr = np.array(_instant)
            f_arr = np.array(_filtered)
            n_tot = _sample_count

        # Rolling window
        t_end = t_arr[-1]
        t_start = max(t_arr[0], t_end - WINDOW_SECONDS)
        mask = t_arr >= t_start

        t_w = t_arr[mask]
        i_w = i_arr[mask]
        f_w = f_arr[mask]
        r_w = i_w - f_w

        line_instant.set_data(t_w, i_w)
        line_filtered.set_data(t_w, f_w)
        line_resid.set_data(t_w, r_w)

        # X axis
        ax_main.set_xlim(t_start, t_end + 0.1)

        # Y axis — main
        if len(i_w):
            lo = min(i_w.min(), f_w.min())
            hi = max(i_w.max(), f_w.max())
            margin = max((hi - lo) * 0.15, 0.05)
            ax_main.set_ylim(lo - margin, hi + margin)

        # Y axis — residual
        if len(r_w):
            r_abs = max(np.abs(r_w).max(), 1e-6)
            ax_resid.set_ylim(-r_abs * 1.5, r_abs * 1.5)

        # Stats
        if len(i_w) > 1:
            fs_hz = (
                1.0 / np.median(np.diff(t_w))
                if len(t_w) > 1 and np.median(np.diff(t_w)) > 0
                else 0.0
            )
            stats_text.set_text(
                "─── WINDOW ───────────\n"
                f"  samples : {len(i_w):>6d}\n"
                f"  total   : {n_tot:>6d}\n"
                f"  rate    : {fs_hz:>5.1f} Hz\n"
                "\n"
                "─── INSTANT ──────────\n"
                f"  mean    : {np.mean(i_w):>+8.4f} kg\n"
                f"  std dev : {np.std(i_w):>8.4f} kg\n"
                f"  min     : {i_w.min():>+8.4f} kg\n"
                f"  max     : {i_w.max():>+8.4f} kg\n"
                f"  p-p     : {i_w.max() - i_w.min():>8.4f} kg\n"
                "\n"
                "─── FILTER RESIDUAL ──\n"
                f"  noise σ : {np.std(r_w):>8.4f} kg\n"
                f"  noise pp: {r_w.max() - r_w.min():>8.4f} kg\n"
                "\n"
                "─── FILTERED ─────────\n"
                f"  mean    : {np.mean(f_w):>+8.4f} kg\n"
            )

        return line_instant, line_filtered, line_resid, stats_text

    return update


# ── Main ──────────────────────────────────────────────────────────────────────


def main() -> None:
    print(f"[graph] connecting to {HOST}:{PORT} …")

    try:
        sock = socket.create_connection((HOST, PORT), timeout=CONNECT_TIMEOUT)
    except socket.timeout:
        print("[graph] timed out — is the Pico running and wifiStreamSensor = true?")
        sys.exit(1)
    except OSError as exc:
        print(f"[graph] could not connect: {exc}")
        sys.exit(1)

    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print("[graph] connected — waiting for first batch (Pico flushes every 300 ms)")

    reader_thread = threading.Thread(target=_reader, args=(sock,), daemon=True)
    reader_thread.start()

    fig, ax_main, ax_resid, line_instant, line_filtered, line_resid, stats_text = (
        _build_figure()
    )

    updater = _make_updater(
        ax_main,
        ax_resid,
        line_instant,
        line_filtered,
        line_resid,
        stats_text,
    )

    ani = animation.FuncAnimation(  # noqa: F841  keep reference alive
        fig,
        updater,
        interval=100,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        _stop_event.set()
        sock.close()
        print("[graph] disconnected.")


if __name__ == "__main__":
    main()
