#!/usr/bin/env python3
"""
Cookstove Data Acquisition Real-Time Serial Reader & Live Plotter
Compatible with the 2 Hz JSONL output stream from tb.cpp.

Features:
  - Auto-detects the microcontroller USB serial port (or accepts --port)
  - Background reader thread prevents serial buffer drops during UI redraws
  - Logs every incoming JSONL record immediately to a local host backup file
  - Gracefully handles JSON `null` values (renders gaps via NaN instead of crashing)
  - Includes a built-in `--demo` simulator mode to test plots without hardware

* AI was used to create the initial template and demo for this document
"""

import argparse
import json
import math
import random
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import serial
import serial.tools.list_ports


# ============================================================================
# 1. SERIAL PORT AUTO-DETECTION & HELPER UTILITIES
# ============================================================================

def find_serial_port() -> str:
    """Scans available USB serial interfaces and returns the best match."""
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError(
            "No USB serial ports detected. Connect the DAQ board or run with --demo."
        )

    # Prefer common USB-to-UART bridge descriptions (CP210x, CH340, FTDI, USB JTAG)
    preferred_keywords = ("usb", "uart", "cp210", "ch340", "ftdi", "acm", "serial")
    for port in ports:
        desc = f"{port.device} {port.description}".lower()
        if any(kw in desc for kw in preferred_keywords):
            return port.device

    return ports[0].device


def to_nan_float(value) -> float:
    """Converts JSON numeric values to float and JSON `null` (None) to np.nan."""
    if value is None:
        return np.nan
    try:
        return float(value)
    except (TypeError, ValueError):
        return np.nan


# ============================================================================
# 2. THREAD-SAFE DATA BUFFER & BACKGROUND SERIAL INGESTION
# ============================================================================

class CookstoveDataBuffer:
    """Thread-safe ring buffer storing incoming 2 Hz sensor records."""

    def __init__(self, max_samples: int | None):
        self.lock = threading.Lock()
        self.t_sec = deque(maxlen=max_samples)
        self.tc_firebox_c = deque(maxlen=max_samples)
        self.tc_water_c = deque(maxlen=max_samples)
        self.co_mv = deque(maxlen=max_samples)
        self.co_raw = deque(maxlen=max_samples)
        self.pm25_ugm3 = deque(maxlen=max_samples)
        self.amb_temp_c = deque(maxlen=max_samples)
        self.amb_rh_pct = deque(maxlen=max_samples)

        self.t0_ms: int | None = None
        self.last_elapsed_s = 0.0
        self.total_packets = 0
        self.present_fields: set[str] = set()

    def append_record(self, record: dict) -> None:
        """Parses one JSON dictionary emitted by tb.cpp and stores each channel."""
        ts_ms = record.get("ts_ms")
        if ts_ms is None:
            return

        with self.lock:
            try:
                ts_ms = int(ts_ms)
            except (TypeError, ValueError):
                return

            self.present_fields.update(
                key for key in record
                if key not in ("ts_ms", "status", "msg")
            )

            if self.t0_ms is None:
                self.t0_ms = ts_ms

            timestamp_delta = ts_ms - self.t0_ms

            # millis() starts over after a firmware reset or long uptime rollover.
            # Rebase only for a large backward jump; small decreases are stale or
            # out-of-order packets and should not reset the elapsed-time origin.
            if timestamp_delta < -1000:
                self.t0_ms = ts_ms
                timestamp_delta = 0
                self.last_elapsed_s = 0.0

            # Elapsed test time in seconds relative to first received packet
            elapsed_s = max(
                self.last_elapsed_s,
                max(0.0, timestamp_delta / 1000.0),
            )
            self.last_elapsed_s = elapsed_s
            self.t_sec.append(elapsed_s)

            self.tc_firebox_c.append(to_nan_float(record.get("tc_firebox_c")))
            self.tc_water_c.append(to_nan_float(record.get("tc_water_c")))
            self.co_mv.append(to_nan_float(record.get("co_mv")))
            self.co_raw.append(to_nan_float(record.get("co_raw")))
            self.pm25_ugm3.append(to_nan_float(record.get("pm25_ugm3")))
            self.amb_temp_c.append(to_nan_float(record.get("amb_temp_c")))
            self.amb_rh_pct.append(to_nan_float(record.get("amb_rh_pct")))

            self.total_packets += 1

    def snapshot(self) -> dict[str, np.ndarray]:
        """Returns a thread-safe NumPy copy of all buffered channels for plotting."""
        with self.lock:
            return {
                "t_sec": np.array(self.t_sec, dtype=float),
                "tc_firebox_c": np.array(self.tc_firebox_c, dtype=float),
                "tc_water_c": np.array(self.tc_water_c, dtype=float),
                "co_mv": np.array(self.co_mv, dtype=float),
                "co_raw": np.array(self.co_raw, dtype=float),
                "pm25_ugm3": np.array(self.pm25_ugm3, dtype=float),
                "amb_temp_c": np.array(self.amb_temp_c, dtype=float),
                "amb_rh_pct": np.array(self.amb_rh_pct, dtype=float),
                "present_fields": set(self.present_fields),
                "total_packets": self.total_packets,
            }


def serial_reader_worker(
    port: str,
    baud: int,
    buffer: CookstoveDataBuffer,
    log_path: Path,
    stop_event: threading.Event,
) -> None:
    """Reads JSONL lines from USB serial, saves them to disk, and updates buffer."""
    print(f"[SERIAL] Connecting to {port} at {baud} baud...")
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=0.5)
        # Flush stale bytes sitting in the OS serial buffer on startup
        ser.reset_input_buffer()
    except serial.SerialException as exc:
        print(f"[ERROR] Could not open {port}: {exc}")
        stop_event.set()
        return

    print(f"[LOGGING] Mirroring raw JSONL stream to: {log_path}")
    with ser, log_path.open("a", encoding="utf-8") as log_file:
        while not stop_event.is_set():
            try:
                raw_bytes = ser.readline()
                if not raw_bytes:
                    continue

                line = raw_bytes.decode("utf-8", errors="replace").strip()
                if not line.startswith("{"):
                    continue

                record = json.loads(line)

                # Surface firmware startup warnings without interrupting data capture
                if record.get("status") == "warn":
                    print(f"[FIRMWARE WARN] {record.get('msg', line)}")
                    continue

                # Write directly to host JSONL file and flush immediately
                log_file.write(line + "\n")
                log_file.flush()

                buffer.append_record(record)

            except json.JSONDecodeError:
                # Ignore partial first line if connected mid-transmission
                continue
            except serial.SerialException as exc:
                print(f"[ERROR] Serial connection lost: {exc}")
                stop_event.set()
                break


def demo_stream_worker(
    buffer: CookstoveDataBuffer,
    log_path: Path,
    stop_event: threading.Event,
) -> None:
    """Generates realistic 2 Hz cookstove test data when --demo is passed."""
    print(f"[DEMO MODE] Simulating 2 Hz JSONL stream -> {log_path}")
    ts_ms = 0
    with log_path.open("a", encoding="utf-8") as log_file:
        while not stop_event.is_set():
            t = ts_ms / 1000.0
            # Simulate ignition ramp, steady boil, and decay
            firebox = 25.0 + 520.0 * (1.0 - math.exp(-t / 45.0)) + random.uniform(-4.0, 4.0)
            water = min(98.5, 21.0 + 0.42 * t + random.uniform(-0.2, 0.2))
            co_mv = 410.0 + 180.0 * math.exp(-((t - 35.0) ** 2) / 900.0) + random.uniform(-3.0, 3.0)
            pm25 = max(2.0, 18.0 + 95.0 * math.exp(-((t - 30.0) ** 2) / 700.0) + random.uniform(-2.5, 2.5))

            record = {
                "ts_ms": ts_ms,
                "amb_temp_c": round(22.5 + 0.01 * t + random.uniform(-0.05, 0.05), 2),
                "amb_rh_pct": round(44.0 - 0.015 * t + random.uniform(-0.1, 0.1), 2),
                "co_raw": int(co_mv / 0.125),
                "co_mv": round(co_mv, 3),
                "pm25_ugm3": round(pm25, 2),
                "tc_firebox_c": round(firebox, 2),
                "tc_water_c": round(water, 2),
            }
            line = json.dumps(record)
            log_file.write(line + "\n")
            log_file.flush()
            buffer.append_record(record)

            ts_ms += 500
            stop_event.wait(0.5)


# ============================================================================
# 3. REAL-TIME 4-PANEL MATPLOTLIB DASHBOARD
# ============================================================================

def format_latest(arr: np.ndarray, unit: str, decimals: int = 1) -> str:
    """Formats the most recent non-NaN reading for subplot title readouts."""
    if arr.size == 0 or np.isnan(arr[-1]):
        return f"-- {unit}"
    return f"{arr[-1]:.{decimals}f} {unit}"


def run_live_dashboard(
    buffer: CookstoveDataBuffer,
    stop_event: threading.Event,
    plot_path: Path,
) -> None:
    """Builds and animates a 2x2 real-time sensor dashboard at 2 Hz."""
    plt.style.use("seaborn-v0_8-darkgrid")
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    fig.canvas.manager.set_window_title("Cookstove DAQ - Live Telemetry (2 Hz)")

    ax_tc = axes[0, 0]
    ax_tc_water = ax_tc.twinx()  # Separate right axis so 100 C water isn't dwarfed by 600 C firebox
    ax_co = axes[0, 1]
    ax_pm = axes[1, 0]
    ax_amb = axes[1, 1]
    ax_amb_rh = ax_amb.twinx()

    # --- Panel 1: Firebox & Water Thermocouples ---
    (line_firebox,) = ax_tc.plot([], [], color="#d95f02", lw=1.8, label="Firebox (°C)")
    (line_water,) = ax_tc_water.plot([], [], color="#1f78b4", lw=1.8, label="Water (°C)")
    ax_tc.set_ylabel("Firebox Temp (°C)", color="#d95f02", fontweight="bold")
    ax_tc_water.set_ylabel("Water Temp (°C)", color="#1f78b4", fontweight="bold")
    ax_tc_water.grid(False)
    ax_tc.legend(
        [line_firebox, line_water],
        ["Firebox (°C)", "Water (°C)"],
        loc="upper left",
        frameon=True,
    )

    # --- Panel 2: Analog CO Sensor (16-bit ADC mV) ---
    (line_co,) = ax_co.plot([], [], color="#7570b3", lw=1.8, label="CO Signal (mV)")
    ax_co.set_ylabel("CO Voltage (mV)", fontweight="bold")
    ax_co.legend(loc="upper left", frameon=True)

    # --- Panel 3: Particulate Matter (PM2.5) ---
    (line_pm25,) = ax_pm.plot([], [], color="#e7298a", lw=1.8, label="PM2.5 (µg/m³)")
    ax_pm.set_ylabel("Concentration (µg/m³)", fontweight="bold")
    ax_pm.set_xlabel("Elapsed Test Time (s)", fontweight="bold")
    ax_pm.legend(loc="upper left", frameon=True)

    # --- Panel 4: Ambient Temperature & Relative Humidity ---
    (line_amb_t,) = ax_amb.plot([], [], color="#e6ab02", lw=1.8, label="Ambient Temp (°C)")
    (line_amb_rh,) = ax_amb_rh.plot([], [], color="#1b9e77", lw=1.6, linestyle="--", label="Rel. Humidity (%)")
    ax_amb.set_ylabel("Ambient Temp (°C)", color="#b58500", fontweight="bold")
    ax_amb_rh.set_ylabel("Rel. Humidity (%)", color="#1b9e77", fontweight="bold")
    ax_amb.set_xlabel("Elapsed Test Time (s)", fontweight="bold")
    ax_amb_rh.grid(False)
    ax_amb.legend(
        [line_amb_t, line_amb_rh],
        ["Ambient Temp (°C)", "Rel. Humidity (%)"],
        loc="upper left",
        frameon=True,
    )

    def rescale_axis(ax, arrays: list[np.ndarray], pad_fraction: float = 0.12) -> None:
        """Auto-scales Y limits cleanly while ignoring NaNs."""
        valid_vals = np.concatenate([a[~np.isnan(a)] for a in arrays if a.size > 0])
        if valid_vals.size == 0:
            return
        ymin, ymax = float(np.min(valid_vals)), float(np.max(valid_vals))
        span = max(ymax - ymin, 1.0)
        ax.set_ylim(ymin - span * pad_fraction, ymax + span * pad_fraction)

    def update_frame(_frame_idx):
        data = buffer.snapshot()
        t = data["t_sec"]
        if t.size == 0:
            return []

        present = data["present_fields"]

        def sensor_status(field: str, arr: np.ndarray, unit: str, decimals: int = 1) -> str:
            if field not in present:
                return "Disabled"
            if arr.size == 0 or not np.any(~np.isnan(arr)):
                return "No valid data"
            return format_latest(arr, unit, decimals)

        # Update line data
        line_firebox.set_data(t, data["tc_firebox_c"])
        line_water.set_data(t, data["tc_water_c"])
        line_co.set_data(t, data["co_mv"])
        line_pm25.set_data(t, data["pm25_ugm3"])
        line_amb_t.set_data(t, data["amb_temp_c"])
        line_amb_rh.set_data(t, data["amb_rh_pct"])

        firebox_present = "tc_firebox_c" in present
        water_present = "tc_water_c" in present
        co_present = "co_mv" in present or "co_raw" in present
        pm25_present = "pm25_ugm3" in present
        amb_temp_present = "amb_temp_c" in present
        amb_rh_present = "amb_rh_pct" in present

        line_firebox.set_visible(firebox_present)
        line_water.set_visible(water_present)
        line_co.set_visible(co_present)
        line_pm25.set_visible(pm25_present)
        line_amb_t.set_visible(amb_temp_present)
        line_amb_rh.set_visible(amb_rh_present)

        ax_tc.set_visible(firebox_present)
        ax_tc_water.set_visible(water_present)
        ax_co.set_visible(co_present)
        ax_pm.set_visible(pm25_present)
        ax_amb.set_visible(amb_temp_present)
        ax_amb_rh.set_visible(amb_rh_present)

        # Update X-axis range across all shared subplots
        xmin = float(t[0])
        xmax = max(float(t[-1]), xmin + 5.0)
        ax_tc.set_xlim(xmin, xmax)

        # Auto-scale each Y axis independently
        rescale_axis(ax_tc, [data["tc_firebox_c"]])
        rescale_axis(ax_tc_water, [data["tc_water_c"]])
        rescale_axis(ax_co, [data["co_mv"]])
        rescale_axis(ax_pm, [data["pm25_ugm3"]])
        rescale_axis(ax_amb, [data["amb_temp_c"]])
        rescale_axis(ax_amb_rh, [data["amb_rh_pct"]])

        # Live numeric HUD in each subplot title
        ax_tc.set_title(
            f"Thermocouples | Firebox: {sensor_status('tc_firebox_c', data['tc_firebox_c'], '°C')}  "
            f"Water: {sensor_status('tc_water_c', data['tc_water_c'], '°C')}",
            fontsize=10,
            fontweight="bold",
        )
        if firebox_present:
            ax_tc_water.set_title(
                f"Firebox: {sensor_status('tc_firebox_c', data['tc_firebox_c'], '°C')}  "
                f"Water: {sensor_status('tc_water_c', data['tc_water_c'], '°C')}",
                fontsize=10,
                fontweight="bold",
            )
        else:
            ax_tc.set_title(
                f"Thermocouples | Firebox: Disabled  "
                f"Water: {sensor_status('tc_water_c', data['tc_water_c'], '°C')}",
                fontsize=10,
                fontweight="bold",
            )

        ax_tc.legend(
            [line for line, enabled in ((line_firebox, firebox_present), (line_water, water_present)) if enabled],
            [label for label, enabled in (
                ("Firebox (°C)", firebox_present),
                ("Water (°C)", water_present),
            ) if enabled],
            loc="upper left",
            frameon=True,
        )
        ax_co.set_title(
            f"CO Sensor | Signal: {sensor_status('co_mv', data['co_mv'], 'mV')} "
            f"(Raw: {sensor_status('co_raw', data['co_raw'], 'cts', 0)})",
            fontsize=10,
            fontweight="bold",
        )
        ax_co.legend(
            [line_co] if co_present else [],
            ["CO Signal (mV)"] if co_present else [],
            loc="upper left",
            frameon=True,
        )
        ax_pm.set_title(
            f"Particulate Matter | PM2.5: {sensor_status('pm25_ugm3', data['pm25_ugm3'], 'µg/m³')}",
            fontsize=10,
            fontweight="bold",
        )
        ax_pm.legend(
            [line_pm25] if pm25_present else [],
            ["PM2.5 (µg/m³)"] if pm25_present else [],
            loc="upper left",
            frameon=True,
        )
        ax_amb.set_title(
            f"Ambient | Temp: {sensor_status('amb_temp_c', data['amb_temp_c'], '°C')}  "
            f"RH: {sensor_status('amb_rh_pct', data['amb_rh_pct'], '%')}",
            fontsize=10,
            fontweight="bold",
        )
        ax_amb.legend(
            [line for line, enabled in ((line_amb_t, amb_temp_present), (line_amb_rh, amb_rh_present)) if enabled],
            [label for label, enabled in (
                ("Ambient Temp (°C)", amb_temp_present),
                ("Rel. Humidity (%)", amb_rh_present),
            ) if enabled],
            loc="upper left",
            frameon=True,
        )

        fig.suptitle(
            f"Cookstove DAQ Live Monitor  |  Samples: {data['total_packets']}  |  Elapsed: {t[-1]:.1f} s",
            fontsize=12,
            fontweight="bold",
        )
        return [
            line_firebox,
            line_water,
            line_co,
            line_pm25,
            line_amb_t,
            line_amb_rh,
        ]

    # 500 ms interval matches the 2 Hz firmware acquisition rate
    _anim = animation.FuncAnimation(
        fig,
        update_frame,
        interval=500,
        blit=False,
        cache_frame_data=False,
    )

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    try:
        plt.show()
    finally:
        plot_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(plot_path, dpi=150, bbox_inches="tight")
        print(f"[PLOT] Saved timestamped plot to: {plot_path}")
        stop_event.set()


# ============================================================================
# 4. ENTRY POINT & CLI ARGUMENTS
# ============================================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Real-time USB serial JSONL reader and plotter for tb.cpp."
    )
    parser.add_argument(
        "--port",
        type=str,
        default="COM3",
        help="Serial port (default: COM3).",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="USB serial baud rate matching USB_SERIAL_BAUD in tb.cpp (default: 115200).",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=300,
        help="Rolling plot window in seconds (default: 300 s; set to 0 for full test).",
    )
    parser.add_argument(
        "--out",
        type=str,
        default=None,
        help="Host backup .jsonl path (default: results/cookstove_test_MM_DD_YYYY_HHMMSS.jsonl).",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Simulate a 2 Hz cookstove test stream without connected hardware.",
    )
    args = parser.parse_args()

    max_samples = None if args.window <= 0 else args.window * 2  # 2 samples/sec
    buffer = CookstoveDataBuffer(max_samples=max_samples)
    stop_event = threading.Event()

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.out:
        log_path = Path(args.out)
    else:
        results_dir = Path(__file__).resolve().parent / "results"
        results_dir.mkdir(parents=True, exist_ok=True)
        log_path = results_dir / f"cookstove_test_{timestamp_str}.jsonl"

    plots_dir = Path(__file__).resolve().parent / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    plot_path = plots_dir / f"daq_plot_{timestamp_str}.png"

    if args.demo:
        worker = threading.Thread(
            target=demo_stream_worker,
            args=(buffer, log_path, stop_event),
            daemon=True,
        )
    else:
        port = args.port or find_serial_port()
        worker = threading.Thread(
            target=serial_reader_worker,
            args=(port, args.baud, buffer, log_path, stop_event),
            daemon=True,
        )

    worker.start()
    try:
        run_live_dashboard(buffer, stop_event, plot_path)
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user. Closing serial stream and saving log...")
    finally:
        stop_event.set()
        worker.join(timeout=2.0)
        print(f"[DONE] Saved {buffer.total_packets} records to {log_path}")


if __name__ == "__main__":
    main()