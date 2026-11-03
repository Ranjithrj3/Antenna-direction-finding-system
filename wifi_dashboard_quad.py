"""
Four-antenna Wi-Fi direction/range dashboard.

Expected serial CSV rows:
    SSID,RSSI_ANT1,RSSI_ANT2,RSSI_ANT3,RSSI_ANT4,BEST

Fixed antenna headings:
    ANT1 = 0 deg (North)
    ANT2 = 90 deg (East)
    ANT3 = 180 deg (South)
    ANT4 = 270 deg (West)

Bearing is estimated from the relative RSSI pattern across all four directional
antennas. Range is estimated from absolute RSSI using a calibratable log-distance
path-loss model, so it is approximate and environment dependent.
"""

from __future__ import annotations

import argparse
import math
import random
import threading
import time
import tkinter as tk
from collections import defaultdict, deque
from tkinter import ttk
from typing import Deque, Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

COM_PORT = "COM5"
BAUD_RATE = 115200
POLL_MS = 200
MAX_POINTS = 100
RSSI_MIN = -100
RSSI_MAX = -20
ACCUM_MAX = 30
ANTENNA_HEADINGS = (0.0, 90.0, 180.0, 270.0)
ANTENNA_NAMES = ("ANT1", "ANT2", "ANT3", "ANT4")
ANTENNA_COLORS = ("#8ab4f8", "#f4a261", "#81c995", "#c58af9")

COLORS = [
    "#e63946", "#457b9d", "#2a9d8f", "#e9c46a", "#f4a261",
    "#9b5de5", "#00bbf9", "#00f5d4", "#fee440", "#f15bb5",
    "#118ab2", "#06d6a0", "#ef476f", "#ffd166", "#c5c9d0",
]

DEMO_TRUE_POS: Dict[str, Tuple[float, float]] = {
    "HomeNet_5G": (35.0, -42.0),
    "CafeWiFi": (118.0, -55.0),
    "OfficeAP": (205.0, -64.0),
    "Neighbor_2G": (302.0, -72.0),
    "IoT_Hub": (82.0, -79.0),
}


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def rssi_to_power(rssi: float) -> float:
    """Convert dBm to linear milliwatt-like relative power."""
    return 10.0 ** (rssi / 10.0)


def rssi_to_size(rssi: float) -> float:
    t = clamp((rssi - RSSI_MIN) / (RSSI_MAX - RSSI_MIN), 0.0, 1.0)
    return 25.0 + 220.0 * t


def pattern_gain(source_deg: float, antenna_deg: float, beamwidth_deg: float, front_back: float) -> float:
    """Directional antenna response in [0.02, 1]."""
    delta = math.radians(((source_deg - antenna_deg + 180.0) % 360.0) - 180.0)
    k = max(0.5, 180.0 / max(beamwidth_deg, 5.0))
    c = math.cos(delta)
    directional = (c * c) ** (k / 2.0) if c > 0 else 0.0
    cardioid = 0.5 * (1.0 + c)
    fb = clamp(front_back, 0.0, 1.0)
    return max(0.02, (1.0 - fb) * cardioid + fb * directional)


def cardioid_radius(theta: np.ndarray, orientation_deg: float, beamwidth_deg: float, front_back: float) -> np.ndarray:
    orient = np.deg2rad(orientation_deg)
    k = max(0.5, 180.0 / max(beamwidth_deg, 5.0))
    delta = theta - orient
    c = np.cos(delta)
    directional = np.where(c > 0, (c ** 2) ** (k / 2.0), 0.0)
    cardioid = 0.5 * (1.0 + c)
    fb = clamp(front_back, 0.0, 1.0)
    return np.clip((1.0 - fb) * cardioid + fb * directional, 0.02, 1.0)


def estimate_distance_m(rssi: float, rssi_at_1m: float, path_loss_n: float) -> float:
    """Log-distance path-loss estimate: d = 10^((A-RSSI)/(10n))."""
    n = max(0.5, path_loss_n)
    return 10.0 ** ((rssi_at_1m - rssi) / (10.0 * n))


def estimate_bearing_4(
    rssis: Tuple[Optional[float], Optional[float], Optional[float], Optional[float]],
    beamwidth_deg: float,
    front_back: float,
) -> Optional[Tuple[float, float]]:
    """Return (bearing_deg, confidence_0_1) from simultaneous four-antenna RSSI.

    For each candidate bearing we fit RSSI = intercept + slope * antenna_gain.
    The intercept absorbs unknown transmitter power/path loss, so the direction is
    determined mainly by the relative four-antenna pattern rather than absolute RSSI.
    """
    valid = [(ANTENNA_HEADINGS[i], float(r)) for i, r in enumerate(rssis) if r is not None]
    if len(valid) < 2:
        return None

    y = np.array([r for _, r in valid], dtype=float)
    bearings = [b for b, _ in valid]
    best_phi = 0.0
    best_rmse = float("inf")

    def fit_rmse(phi: float) -> float:
        x = np.array([pattern_gain(phi, b, beamwidth_deg, front_back) for b in bearings], dtype=float)
        X = np.column_stack((np.ones_like(x), x))
        beta, *_ = np.linalg.lstsq(X, y, rcond=None)
        # A directional antenna should get stronger toward boresight. Penalize inverted fits.
        if beta[1] < 0:
            return 1e6 + abs(float(beta[1]))
        pred = X @ beta
        return float(np.sqrt(np.mean((y - pred) ** 2)))

    for phi in np.arange(0.0, 360.0, 1.0):
        err = fit_rmse(float(phi))
        if err < best_rmse:
            best_rmse = err
            best_phi = float(phi)

    for phi in np.arange(best_phi - 1.0, best_phi + 1.01, 0.1):
        p = float(phi % 360.0)
        err = fit_rmse(p)
        if err < best_rmse:
            best_rmse = err
            best_phi = p

    spread_db = float(y.max() - y.min())
    fit_conf = clamp(1.0 - best_rmse / 5.0, 0.0, 1.0)
    contrast_conf = clamp(spread_db / 18.0, 0.0, 1.0)
    count_conf = len(valid) / 4.0
    confidence = 0.55 * fit_conf + 0.30 * contrast_conf + 0.15 * count_conf
    return best_phi % 360.0, clamp(confidence, 0.0, 1.0)


class SerialReader:
    def __init__(self, port: str, baud: int, demo: bool = False, pattern_fn=None):
        self.port = port
        self.baud = baud
        self.demo = demo
        self.pattern_fn = pattern_fn or (lambda: (70.0, 0.75))
        self._lock = threading.Lock()
        self._buffer: List[Tuple[str, Optional[int], Optional[int], Optional[int], Optional[int], str]] = []
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._ser: Optional[serial.Serial] = None
        self.status = "Disconnected"
        self.error: Optional[str] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._demo_loop if self.demo else self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        if self._thread:
            self._thread.join(timeout=1.5)

    def drain(self):
        with self._lock:
            items = list(self._buffer)
            self._buffer.clear()
        return items

    def _demo_loop(self) -> None:
        self.status = "Demo mode (4 antennas)"
        t0 = time.time()
        while self._running:
            elapsed = time.time() - t0
            bw, fb = self.pattern_fn()
            batch = []
            for name, (true_brg, peak_rssi) in DEMO_TRUE_POS.items():
                vals = []
                common = 0.8 * math.sin(elapsed * 0.7 + (hash(name) % 9))
                for heading in ANTENNA_HEADINGS:
                    g = pattern_gain(true_brg, heading, bw, fb)
                    noise = random.uniform(-0.8, 0.8)
                    rssi = int(clamp(peak_rssi - 28.0 * (1.0 - g) + common + noise, RSSI_MIN, RSSI_MAX))
                    vals.append(rssi)
                best_i = int(np.argmax(vals))
                batch.append((name, vals[0], vals[1], vals[2], vals[3], ANTENNA_NAMES[best_i]))
            with self._lock:
                self._buffer.extend(batch)
            time.sleep(0.35)

    def _loop(self) -> None:
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
            self.status = f"Connected ({self.port})"
            self.error = None
        except Exception as exc:
            self.status = "Error"
            self.error = str(exc)
            self._running = False
            return

        header = "SSID,RSSI_ANT1,RSSI_ANT2,RSSI_ANT3,RSSI_ANT4,BEST"
        while self._running:
            try:
                if not self._ser or not self._ser.in_waiting:
                    time.sleep(0.01)
                    continue
                line = self._ser.readline().decode(errors="ignore").strip()
                if not line or line.startswith("=") or line == header:
                    continue
                parts = [p.strip() for p in line.split(",")]
                if len(parts) != 6:
                    continue
                ssid, a1s, a2s, a3s, a4s, best = parts
                if not ssid:
                    continue

                def parse(v: str) -> Optional[int]:
                    if v.upper() == "NA":
                        return None
                    try:
                        return int(v)
                    except ValueError:
                        return None

                vals = (parse(a1s), parse(a2s), parse(a3s), parse(a4s))
                if all(v is None for v in vals):
                    continue
                if best not in set(ANTENNA_NAMES) | {"EQUAL"}:
                    present = [(i, v) for i, v in enumerate(vals) if v is not None]
                    best = ANTENNA_NAMES[max(present, key=lambda x: x[1])[0]] if present else "EQUAL"
                with self._lock:
                    self._buffer.append((ssid, vals[0], vals[1], vals[2], vals[3], best))
            except Exception:
                continue
        self.status = "Disconnected"


class WifiDashboard(tk.Tk):
    def __init__(self, port: str = COM_PORT, baud: int = BAUD_RATE, demo: bool = False) -> None:
        super().__init__()
        self.title("4-Antenna Wi-Fi Direction & Range Tracker")
        self.geometry("1500x980")
        self.minsize(1200, 780)
        self.configure(bg="#1a1d23")

        self.beamwidth = tk.DoubleVar(value=70.0)
        self.front_back = tk.DoubleVar(value=0.75)
        self.rssi_at_1m = tk.DoubleVar(value=-42.0)
        self.path_loss_n = tk.DoubleVar(value=2.4)
        self.max_range_m = tk.DoubleVar(value=50.0)

        self.history: Dict[str, Deque[int]] = defaultdict(lambda: deque(maxlen=MAX_POINTS))
        self.latest: Dict[str, int] = {}
        self.latest_ants: Dict[str, Tuple[Optional[int], Optional[int], Optional[int], Optional[int]]] = {}
        self.filtered_ants: Dict[str, Tuple[float, float, float, float]] = {}
        self._ant_samples: Dict[str, List[Deque[int]]] = defaultdict(lambda: [deque(maxlen=ACCUM_MAX) for _ in range(4)])
        self.best_antenna: Dict[str, str] = {}
        self.estimates: Dict[str, Tuple[float, float, float]] = {}  # bearing, meters, confidence
        self.visible: Dict[str, tk.BooleanVar] = {}
        self.colors: Dict[str, str] = {}
        self._list_widgets: Dict[str, ttk.Frame] = {}
        self._color_idx = 0

        self.reader = SerialReader(port, baud, demo=demo, pattern_fn=lambda: (self.beamwidth.get(), self.front_back.get()))
        self._build_styles()
        self._build_layout()
        self._build_plots()
        self._bind_controls()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.reader.start()
        self.after(POLL_MS, self._tick)

    def _build_styles(self) -> None:
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TFrame", background="#1a1d23")
        style.configure("Panel.TFrame", background="#242830")
        style.configure("TLabel", background="#1a1d23", foreground="#e8eaed", font=("Segoe UI", 10))
        style.configure("Title.TLabel", background="#242830", foreground="#f0f3f7", font=("Segoe UI Semibold", 12))
        style.configure("Muted.TLabel", background="#242830", foreground="#9aa0a6", font=("Segoe UI", 9))
        style.configure("Metric.TLabel", background="#242830", foreground="#8ab4f8", font=("Consolas", 10))
        style.configure("Status.TLabel", background="#1a1d23", foreground="#9aa0a6", font=("Segoe UI", 9))
        style.configure("TCheckbutton", background="#242830", foreground="#e8eaed", font=("Segoe UI", 9))
        style.configure("TLabelframe", background="#242830", foreground="#e8eaed")
        style.configure("TLabelframe.Label", background="#242830", foreground="#e8eaed", font=("Segoe UI Semibold", 10))
        style.configure("Horizontal.TScale", background="#242830")

    def _build_layout(self) -> None:
        root = ttk.Frame(self)
        root.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(1, weight=3)
        root.rowconfigure(2, weight=2)

        header = ttk.Frame(root)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ttk.Label(header, text="4-Antenna Wi-Fi Tracker · N / E / S / W", font=("Segoe UI Semibold", 16)).pack(side=tk.LEFT)
        self.status_label = ttk.Label(header, text="Connecting...", style="Status.TLabel")
        self.status_label.pack(side=tk.RIGHT)

        top = ttk.Frame(root)
        top.grid(row=1, column=0, sticky="nsew")
        top.columnconfigure(0, weight=2)
        top.columnconfigure(1, weight=1)
        top.rowconfigure(0, weight=1)

        self.radar_host = ttk.Frame(top, style="Panel.TFrame")
        self.radar_host.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.ctrl_host = ttk.Frame(top, style="Panel.TFrame")
        self.ctrl_host.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        self._build_controls(self.ctrl_host)

        bottom = ttk.Frame(root)
        bottom.grid(row=2, column=0, sticky="nsew", pady=(10, 0))
        bottom.columnconfigure(0, weight=2)
        bottom.columnconfigure(1, weight=1)
        bottom.rowconfigure(0, weight=1)

        self.plot_host = ttk.Frame(bottom, style="Panel.TFrame")
        self.plot_host.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.plot_host.columnconfigure(0, weight=1)
        self.plot_host.rowconfigure(0, weight=1)
        self.list_host = ttk.Frame(bottom, style="Panel.TFrame")
        self.list_host.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        self.list_host.columnconfigure(0, weight=1)
        self.list_host.rowconfigure(3, weight=1)
        self._build_signal_list(self.list_host)

    def _add_scale(self, parent, title, var, lo, hi, fmt):
        ttk.Label(parent, text=title, style="Muted.TLabel").pack(anchor="w", padx=12, pady=(7, 2))
        row = ttk.Frame(parent, style="Panel.TFrame")
        row.pack(fill=tk.X, padx=12)
        ttk.Scale(row, from_=lo, to=hi, orient=tk.HORIZONTAL, variable=var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        lab = ttk.Label(row, text=fmt(var.get()), style="Metric.TLabel", width=8)
        lab.pack(side=tk.RIGHT, padx=(8, 0))
        var.trace_add("write", lambda *_: lab.configure(text=fmt(var.get())))

    def _build_controls(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="Fixed 4-Antenna Array", style="Title.TLabel").pack(anchor="w", padx=12, pady=(12, 8))
        for name, deg, color in zip(ANTENNA_NAMES, ANTENNA_HEADINGS, ANTENNA_COLORS):
            ttk.Label(parent, text=f"{name}: {deg:.0f} deg", style="Metric.TLabel").pack(anchor="w", padx=18, pady=1)

        ttk.Separator(parent, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=12, pady=10)
        self._add_scale(parent, "Directional beamwidth (deg)", self.beamwidth, 20, 160, lambda v: f"{v:.0f}")
        self._add_scale(parent, "Front/back shaping", self.front_back, 0.0, 1.0, lambda v: f"{v:.2f}")

        range_box = ttk.LabelFrame(parent, text="Range calibration", padding=8)
        range_box.pack(fill=tk.X, padx=12, pady=12)
        self._add_scale(range_box, "RSSI at 1 m (dBm)", self.rssi_at_1m, -70, -25, lambda v: f"{v:.0f}")
        self._add_scale(range_box, "Path-loss exponent n", self.path_loss_n, 1.2, 5.0, lambda v: f"{v:.2f}")
        self._add_scale(range_box, "Radar max range (m)", self.max_range_m, 5, 200, lambda v: f"{v:.0f}")

        ttk.Label(
            parent,
            text=("Bearing: relative pattern fit using ANT1..ANT4 simultaneously.\n"
                  "Range: RSSI log-distance estimate; calibrate A and n for your site.\n"
                  "For true geometric triangulation, antenna phase/TDoA or spatially separated receivers are required."),
            style="Muted.TLabel", wraplength=310, justify=tk.LEFT,
        ).pack(anchor="w", padx=12, pady=(2, 12))

    def _build_signal_list(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="Detected Signals", style="Title.TLabel").grid(row=0, column=0, columnspan=2, sticky="w", padx=12, pady=(12, 4))
        ttk.Label(parent, text="A1/A2/A3/A4 dBm · bearing · range", style="Muted.TLabel").grid(row=1, column=0, columnspan=2, sticky="w", padx=12)
        btn = ttk.Frame(parent, style="Panel.TFrame")
        btn.grid(row=2, column=0, columnspan=2, sticky="ew", padx=12, pady=6)
        ttk.Button(btn, text="Show all", command=lambda: self._set_all(True)).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Button(btn, text="Hide all", command=lambda: self._set_all(False)).pack(side=tk.LEFT)

        canvas = tk.Canvas(parent, bg="#242830", highlightthickness=0)
        scrollbar = ttk.Scrollbar(parent, orient=tk.VERTICAL, command=canvas.yview)
        self.list_inner = ttk.Frame(canvas, style="Panel.TFrame")
        self.list_inner.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        win = canvas.create_window((0, 0), window=self.list_inner, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(win, width=e.width))
        canvas.grid(row=3, column=0, sticky="nsew", padx=(12, 0), pady=(0, 12))
        scrollbar.grid(row=3, column=1, sticky="ns", padx=(0, 8), pady=(0, 12))

    def _build_plots(self) -> None:
        self.fig_radar = Figure(figsize=(6.5, 4.5), dpi=100, facecolor="#242830")
        self.ax_radar = self.fig_radar.add_subplot(111, projection="polar", facecolor="#1a1d23")
        self.canvas_radar = FigureCanvasTkAgg(self.fig_radar, master=self.radar_host)
        self.canvas_radar.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        self.fig_plot = Figure(figsize=(6, 2.8), dpi=100, facecolor="#242830")
        self.ax_plot = self.fig_plot.add_subplot(111, facecolor="#1a1d23")
        self.canvas_plot = FigureCanvasTkAgg(self.fig_plot, master=self.plot_host)
        self.canvas_plot.get_tk_widget().grid(row=0, column=0, sticky="nsew", padx=4, pady=(4, 0))
        toolbar_frame = ttk.Frame(self.plot_host, style="Panel.TFrame")
        toolbar_frame.grid(row=1, column=0, sticky="ew")
        NavigationToolbar2Tk(self.canvas_plot, toolbar_frame).update()
        self._redraw_all()

    def _bind_controls(self) -> None:
        for var in (self.beamwidth, self.front_back, self.rssi_at_1m, self.path_loss_n, self.max_range_m):
            var.trace_add("write", lambda *_: self._on_calibration_change())

    def _on_calibration_change(self) -> None:
        self._recompute_all_estimates()
        self._update_list_labels()
        self._redraw_all()

    def _ensure_ssid(self, ssid: str) -> None:
        if ssid in self.visible:
            return
        color = COLORS[self._color_idx % len(COLORS)]
        self._color_idx += 1
        self.colors[ssid] = color
        var = tk.BooleanVar(value=True)
        self.visible[ssid] = var
        row = ttk.Frame(self.list_inner, style="Panel.TFrame")
        row.pack(fill=tk.X, padx=4, pady=2)
        swatch = tk.Canvas(row, width=12, height=12, bg="#242830", highlightthickness=0)
        swatch.create_oval(1, 1, 11, 11, fill=color, outline=color)
        swatch.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Checkbutton(row, text=ssid, variable=var, command=self._redraw_all).pack(side=tk.LEFT, fill=tk.X, expand=True)
        est_lab = ttk.Label(row, text="", style="Muted.TLabel", width=18)
        est_lab.pack(side=tk.RIGHT, padx=(6, 0))
        rssi_lab = ttk.Label(row, text="-/-/-/-", style="Metric.TLabel", width=21)
        rssi_lab.pack(side=tk.RIGHT)
        row.rssi_label = rssi_lab  # type: ignore[attr-defined]
        row.est_label = est_lab  # type: ignore[attr-defined]
        self._list_widgets[ssid] = row

    def _set_all(self, value: bool) -> None:
        for v in self.visible.values():
            v.set(value)
        self._redraw_all()

    def _ingest(self, rows) -> None:
        for ssid, a1, a2, a3, a4, best in rows:
            self._ensure_ssid(ssid)
            vals = (a1, a2, a3, a4)
            self.latest_ants[ssid] = vals
            self.best_antenna[ssid] = best
            present = [v for v in vals if v is not None]
            if not present:
                continue
            strongest = max(present)
            self.latest[ssid] = strongest
            self.history[ssid].append(strongest)

            filtered = []
            for i, value in enumerate(vals):
                if value is not None:
                    self._ant_samples[ssid][i].append(value)
                samples = self._ant_samples[ssid][i]
                filtered.append(float(sum(samples) / len(samples)) if samples else float(RSSI_MIN))
            self.filtered_ants[ssid] = tuple(filtered)  # type: ignore[assignment]
            self._recompute_estimate(ssid)
        self._update_list_labels()

    def _recompute_estimate(self, ssid: str) -> None:
        raw = self.latest_ants.get(ssid)
        if not raw:
            return
        smooth = self.filtered_ants.get(ssid)
        if not smooth:
            return
        # Preserve missing channels when passing to the fit.
        fit_vals = tuple(smooth[i] if raw[i] is not None else None for i in range(4))
        brg = estimate_bearing_4(fit_vals, self.beamwidth.get(), self.front_back.get())
        if brg is None:
            self.estimates.pop(ssid, None)
            return
        bearing, confidence = brg

        # Correct strongest measured RSSI for the modeled gain at the estimated bearing.
        corrected_candidates = []
        for i, r in enumerate(smooth):
            if raw[i] is None:
                continue
            g = max(pattern_gain(bearing, ANTENNA_HEADINGS[i], self.beamwidth.get(), self.front_back.get()), 0.02)
            gain_db = 10.0 * math.log10(g)
            corrected_candidates.append(r - gain_db)
        corrected_rssi = max(corrected_candidates) if corrected_candidates else max(smooth)
        distance = estimate_distance_m(corrected_rssi, self.rssi_at_1m.get(), self.path_loss_n.get())
        self.estimates[ssid] = (bearing, distance, confidence)

    def _recompute_all_estimates(self) -> None:
        for ssid in list(self.latest_ants):
            self._recompute_estimate(ssid)

    def _update_list_labels(self) -> None:
        for ssid, row in self._list_widgets.items():
            vals = self.latest_ants.get(ssid, (None, None, None, None))
            text = "/".join("NA" if v is None else str(v) for v in vals)
            row.rssi_label.configure(text=text)  # type: ignore[attr-defined]
            est = self.estimates.get(ssid)
            if est:
                b, d, c = est
                row.est_label.configure(text=f"{b:03.0f} deg  {d:5.1f} m  {c*100:2.0f}%")  # type: ignore[attr-defined]
            else:
                row.est_label.configure(text="no fix")  # type: ignore[attr-defined]

    def _style_radar(self) -> None:
        ax = self.ax_radar
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
        ax.set_ylim(0, 1.05)
        ax.tick_params(colors="#9aa0a6", labelsize=8)
        ax.spines["polar"].set_color("#3c4048")
        ax.grid(color="#3c4048", linestyle="--", linewidth=0.6)
        ax.set_thetagrids([0, 90, 180, 270], labels=["N / ANT1", "E / ANT2", "S / ANT3", "W / ANT4"], color="#f0f3f7")
        ax.set_rgrids([0.25, 0.5, 0.75, 1.0], labels=[])
        ax.set_title("Direction + estimated range", color="#f0f3f7", pad=14, fontsize=11)

    def _draw_radar(self) -> None:
        ax = self.ax_radar
        ax.clear()
        self._style_radar()
        theta = np.linspace(0, 2 * np.pi, 360)
        for name, heading, color in zip(ANTENNA_NAMES, ANTENNA_HEADINGS, ANTENNA_COLORS):
            r = cardioid_radius(theta, heading, self.beamwidth.get(), self.front_back.get())
            ax.plot(theta, r, color=color, lw=1.4, alpha=0.75)
            ax.fill(theta, r, color=color, alpha=0.045)
            a = math.radians(heading)
            ax.annotate("", xy=(a, 1.0), xytext=(a, 0.0), arrowprops=dict(arrowstyle="->", color=color, lw=1.5))
            ax.text(a, 1.02, name, color=color, fontsize=8, ha="center", va="bottom")

        max_range = max(1.0, self.max_range_m.get())
        for ssid, est in self.estimates.items():
            if not self.visible.get(ssid) or not self.visible[ssid].get():
                continue
            bearing, distance, confidence = est
            theta_s = math.radians(bearing)
            radius = clamp(distance / max_range, 0.03, 1.0)
            color = self.colors[ssid]
            size = rssi_to_size(self.latest.get(ssid, RSSI_MIN))
            ax.scatter([theta_s], [radius], s=size * 1.35, facecolors="none", edgecolors=color, linewidths=1.2, alpha=0.45)
            ax.scatter([theta_s], [radius], s=size, c=color, edgecolors="white", linewidths=0.7, alpha=0.55 + 0.4 * confidence)
            ax.plot([theta_s, theta_s], [0.0, radius], color=color, alpha=0.2, lw=1.0)
            ax.annotate(f"{ssid}\n{bearing:.0f} deg / {distance:.1f} m", (theta_s, min(radius + 0.08, 1.02)), color=color, fontsize=7, ha="center")

        # Label physical range rings.
        for frac in (0.25, 0.5, 0.75, 1.0):
            ax.text(math.radians(20), frac, f"{max_range*frac:.0f}m", color="#6f7680", fontsize=7)

    def _draw_intensity(self) -> None:
        ax = self.ax_plot
        ax.clear()
        ax.set_facecolor("#1a1d23")
        ax.tick_params(colors="#9aa0a6", labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#3c4048")
        ax.set_ylabel("Best RSSI (dBm)", color="#c5c9d0")
        ax.set_xlabel("Sample", color="#c5c9d0")
        ax.set_title("Signal Intensity", color="#f0f3f7", fontsize=11)
        ax.set_ylim(RSSI_MIN, RSSI_MAX)
        ax.grid(True, color="#3c4048", linestyle="--", linewidth=0.6)
        any_plot = False
        for ssid, values in self.history.items():
            if not self.visible.get(ssid) or not self.visible[ssid].get() or not values:
                continue
            ys = list(values)
            xs = list(range(len(ys)))
            color = self.colors[ssid]
            ax.plot(xs, ys, color=color, linewidth=1.5, alpha=0.85, label=ssid)
            any_plot = True
        if any_plot:
            ax.legend(loc="upper right", fontsize=8, facecolor="#242830", edgecolor="#3c4048", labelcolor="#e8eaed")
        self.fig_plot.tight_layout()

    def _redraw_all(self) -> None:
        if not hasattr(self, "ax_radar"):
            return
        self._draw_radar()
        self._draw_intensity()
        self.canvas_radar.draw_idle()
        self.canvas_plot.draw_idle()

    def _tick(self) -> None:
        if self.reader.error:
            self.status_label.configure(text=f"Serial error: {self.reader.error}")
        else:
            self.status_label.configure(text=f"{self.reader.status} · 4 live antennas · {len(self.estimates)} fixes")
        rows = self.reader.drain()
        if rows:
            self._ingest(rows)
            self._redraw_all()
        self.after(POLL_MS, self._tick)

    def _on_close(self) -> None:
        self.reader.stop()
        self.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="Four-antenna Wi-Fi direction/range dashboard")
    parser.add_argument("--port", default=COM_PORT, help=f"Serial port (default: {COM_PORT})")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help=f"Baud rate (default: {BAUD_RATE})")
    parser.add_argument("--demo", action="store_true", help="Run with simulated four-antenna Wi-Fi data")
    args = parser.parse_args()

    plt.rcParams.update({"font.family": "Segoe UI", "axes.titleweight": "medium"})
    app = WifiDashboard(port=args.port, baud=args.baud, demo=args.demo)
    app.mainloop()


if __name__ == "__main__":
    main()
