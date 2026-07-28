import argparse
import csv
import json
import math
from pathlib import Path
from tkinter import Tk, filedialog

import matplotlib.pyplot as plt
import numpy as np

# ----------------------------- Parameters ------------------------------------

DATA_DIR = r"D:\yixianglin\Desktop\PHRI_Data"
CALIBRATION_DIR = r"D:\Unity Projects\Rover\Assets\Test\Calibration"

# Use "weighted" to match InteractionEstimator WeightedCoContraction:
#   P=sum(w>0 |w|u), N=sum(w<0 |w|u), spi=2*min(P,N)
# Use "pair" to ignore weights and use channel pairs (0,1), (2,3), (4,5).
SPI_METHOD = "weighted"

EMG_COLUMN_PREFIX = "envelope_ch"

DOWNSAMPLE_HZ = 100.0
USE_SPI_TRIANGULAR_FILTER = True
SPI_TRIANGULAR_WINDOW_MS = 100.0
SPI_EMA_TAU_MS = 100.0

# Plot window. Set duration <= 0 to plot all.
PLOT_START_SEC = 0.0
PLOT_DURATION_SEC = 30.0

# If True, also plot normalized stiffness command proxy in [0, 1].
PLOT_SPI_NORM = False


def choose_file(title, initial_dir, patterns):
    root = Tk()
    root.withdraw()
    path = filedialog.askopenfilename(
        title=title,
        initialdir=initial_dir,
        filetypes=patterns,
    )
    root.destroy()
    return Path(path) if path else None


def latest_file(directory, pattern):
    files = sorted(Path(directory).glob(pattern), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


def parse_channel(name):
    return int(name.rsplit("ch", 1)[1])


def read_emg_csv(path):
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        emg_cols = sorted(
            [name for name in fields if name.strip().lower().startswith(EMG_COLUMN_PREFIX.lower())],
            key=parse_channel,
        )
        if "global_t" not in fields:
            raise ValueError(f"No global_t column in {path}.")
        if not emg_cols:
            raise ValueError(f"No {EMG_COLUMN_PREFIX}* columns in {path}.")

        times = []
        rows = []
        sec = []
        for row in reader:
            try:
                times.append(float(row["global_t"]))
                sec.append(int(float(row.get("sec", 0) or 0)))
                rows.append([float(row[col]) for col in emg_cols])
            except (TypeError, ValueError):
                continue

    if len(times) < 4:
        raise ValueError(f"Not enough EMG samples in {path}.")

    t = np.asarray(times, dtype=float)
    y = np.asarray(rows, dtype=float)
    t = t - t[0]
    return t, np.asarray(sec, dtype=int), y, emg_cols


def load_calibration(path):
    with open(path, "r", encoding="utf-8-sig") as f:
        return json.load(f)


def moving_average(values, window_samples):
    if window_samples <= 1:
        return values
    kernel = np.ones(window_samples, dtype=float) / float(window_samples)
    out = np.empty_like(values)
    for i in range(values.shape[1]):
        out[:, i] = np.convolve(values[:, i], kernel, mode="same")
    return out


def downsample(t, values, target_hz):
    if target_hz <= 0 or len(t) < 2:
        return t, values
    period = 1.0 / target_hz

    # Match DelsysEMG.ImuEmgThreadRoutine:
    # downSampleAccumulator += samplingInterval; publish when accumulator >= period;
    # downSampleAccumulator %= period.
    dt = float(np.median(np.diff(t)))
    if not np.isfinite(dt) or dt <= 0:
        dt = period
    keep = []
    acc = 0.0
    for i in range(len(t)):
        acc += dt
        if acc + 1e-12 >= period:
            keep.append(i)
            acc %= period
    if not keep:
        keep = [0]
    idx = np.asarray(keep, dtype=int)
    return t[idx], values[idx]


def normalize_emg(emg, rest, ref):
    count = min(emg.shape[1], len(rest), len(ref))
    if count <= 0:
        raise ValueError("Calibration rest/ref do not match EMG channels.")
    rest_arr = np.asarray(rest[:count], dtype=float)
    ref_arr = np.asarray(ref[:count], dtype=float)
    denom = ref_arr - rest_arr
    valid = np.abs(denom) > 1e-6
    u = np.zeros((emg.shape[0], count), dtype=float)
    if np.any(valid):
        u[:, valid] = (emg[:, :count][:, valid] - rest_arr[valid]) / denom[valid]
    return np.clip(u, 0.0, 1.0), valid


def weighted_spi(u, weights):
    count = min(u.shape[1], len(weights))
    w = np.asarray(weights[:count], dtype=float)
    pos_mask = w > 0
    neg_mask = w < 0
    aw = np.abs(w)
    p = np.sum(u[:, :count][:, pos_mask] * aw[pos_mask], axis=1) if np.any(pos_mask) else np.zeros(len(u))
    n = np.sum(u[:, :count][:, neg_mask] * aw[neg_mask], axis=1) if np.any(neg_mask) else np.zeros(len(u))
    old_spi = np.sum(u[:, :count] * aw, axis=1)
    co_spi = p + n - np.abs(p - n)
    return co_spi, old_spi, p, n


def pair_spi(u):
    pairs = [(0, 1), (2, 3), (4, 5)]
    spi = np.zeros(len(u), dtype=float)
    count = np.zeros(len(u), dtype=float)
    for a, b in pairs:
        if a >= u.shape[1] or b >= u.shape[1]:
            continue
        high = np.maximum(u[:, a], u[:, b])
        low = np.minimum(u[:, a], u[:, b])
        valid = high >= 1e-4
        spi[valid] += (low[valid] / high[valid]) * (low[valid] + high[valid])
        count[valid] += 1.0
    out = np.zeros(len(u), dtype=float)
    valid = count > 0
    out[valid] = spi[valid] / count[valid]
    return out


def triangular_filter(t, x, window_ms):
    if window_ms <= 0 or len(x) < 2:
        return x.copy()
    dt = float(np.median(np.diff(t)))
    size = max(1, int(round((window_ms / 1000.0) / max(dt, 1e-6))))
    out = np.empty_like(x)
    buf = []
    for i, value in enumerate(x):
        buf.append(value)
        if len(buf) > size:
            buf.pop(0)
        weights = np.arange(1, len(buf) + 1, dtype=float)
        out[i] = float(np.dot(buf, weights) / np.sum(weights))
    return out


def ema_filter(t, x, tau_ms):
    if tau_ms <= 0 or len(x) < 2:
        return x.copy()
    tau = tau_ms / 1000.0
    out = np.empty_like(x)
    out[0] = x[0]
    for i in range(1, len(x)):
        dt = max(float(t[i] - t[i - 1]), 1e-6)
        alpha = 1.0 - math.exp(-dt / max(tau, 1e-6))
        out[i] = out[i - 1] + alpha * (x[i] - out[i - 1])
    return out


def postprocess_spi(t, spi):
    tri = spi.copy()
    if USE_SPI_TRIANGULAR_FILTER:
        tri = triangular_filter(t, tri, SPI_TRIANGULAR_WINDOW_MS)
    ema = ema_filter(t, tri, SPI_EMA_TAU_MS)
    return tri, ema


def plot_results(t, raw_spi, tri_spi, ema_spi, profile, title):
    mask = t >= PLOT_START_SEC
    if PLOT_DURATION_SEC > 0:
        mask &= t <= PLOT_START_SEC + PLOT_DURATION_SEC
    if not np.any(mask):
        mask = np.ones_like(t, dtype=bool)

    row_count = 3 if PLOT_SPI_NORM else 2
    fig, axes = plt.subplots(row_count, 1, sharex=True, figsize=(12, 8))
    if not isinstance(axes, np.ndarray):
        axes = np.asarray([axes])

    ax = axes[0]
    ax.plot(t[mask], raw_spi[mask], label="SPI raw", linewidth=1.2)
    ax.plot(t[mask], ema_spi[mask], label="SPI after triangular + EMA", linewidth=1.8)
    ax.set_ylabel("SPI")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    ax.set_title(title)

    ax = axes[1]
    ax.plot(t[mask], raw_spi[mask], label="SPI raw", linewidth=1.0)
    ax.plot(t[mask], tri_spi[mask], label="SPI after triangular", linewidth=1.2)
    ax.plot(t[mask], ema_spi[mask], label="SPI after EMA", linewidth=1.6)
    ax.set_ylabel("SPI stages")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    if PLOT_SPI_NORM:
        rest = float(profile.get("spi_rest", 0.0))
        ref = float(profile.get("spi_ref", rest + 1.0))
        denom = max(ref - rest, 1e-6)
        raw_norm = np.clip((raw_spi - rest) / denom, 0.0, 1.0)
        ema_norm = np.clip((ema_spi - rest) / denom, 0.0, 1.0)
        axes[2].plot(t[mask], raw_norm[mask], label="SPI norm raw")
        axes[2].plot(t[mask], ema_norm[mask], label="SPI norm postprocessed")
        axes[2].set_ylabel("SPI norm")
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc="upper right")

    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Compare raw SPI and postprocessed SPI from Unity EmgLogs.")
    parser.add_argument("--emg", type=Path, help="Path to EmgLogs CSV. If omitted, a file picker opens.")
    parser.add_argument("--calib", type=Path, help="Path to P*_calibration.json. If omitted, a file picker opens.")
    parser.add_argument("--latest-emg", action="store_true", help="Use latest EmgLogs*.csv instead of opening the EMG picker.")
    args = parser.parse_args()

    emg_path = args.emg
    if emg_path is None and args.latest_emg:
        emg_path = latest_file(DATA_DIR, "EmgLogs*.csv")
    if emg_path is None:
        emg_path = choose_file("Select EmgLogs CSV", DATA_DIR, [("EmgLogs CSV", "EmgLogs*.csv"), ("CSV files", "*.csv")])
    if emg_path is None:
        raise SystemExit("No EmgLogs CSV selected.")

    calib_path = args.calib
    if calib_path is None:
        calib_path = choose_file(
            "Select P*_calibration.json",
            CALIBRATION_DIR,
            [("Calibration JSON", "P*_calibration.json"), ("JSON files", "*.json")],
        )
    if calib_path is None:
        raise SystemExit("No calibration JSON selected.")

    profile = load_calibration(calib_path)
    t, _sec, emg, columns = read_emg_csv(emg_path)
    t, emg = downsample(t, emg, DOWNSAMPLE_HZ)
    u, valid_norm = normalize_emg(emg, profile.get("emg_rest", []), profile.get("emg_ref", []))

    weights = profile.get("emg_force_weights", [])
    if SPI_METHOD.lower() == "weighted":
        if not weights or not any(abs(float(w)) > 0 for w in weights):
            print("Warning: no valid weights. Falling back to pair SPI.")
            raw_spi = pair_spi(u)
        else:
            raw_spi, _old_spi, _p, _n = weighted_spi(u, weights)
    elif SPI_METHOD.lower() == "pair":
        raw_spi = pair_spi(u)
    else:
        raise ValueError(f"Unknown SPI_METHOD: {SPI_METHOD}")

    tri_spi, ema_spi = postprocess_spi(t, raw_spi)

    print(f"EMG file: {emg_path}")
    print(f"Calibration: {calib_path}")
    print(f"Columns: {', '.join(columns)}")
    print(f"Samples after downsample: {len(t)} at target {DOWNSAMPLE_HZ:g} Hz")
    print(f"Valid norm channels: {np.where(valid_norm)[0].tolist()}")
    print(f"SPI method: {SPI_METHOD}")
    print(f"SPI triangular: {USE_SPI_TRIANGULAR_FILTER}, window={SPI_TRIANGULAR_WINDOW_MS:g} ms")
    print(f"SPI EMA tau: {SPI_EMA_TAU_MS:g} ms")

    title = f"{emg_path.name} | {SPI_METHOD} | tri={SPI_TRIANGULAR_WINDOW_MS:g}ms ema={SPI_EMA_TAU_MS:g}ms"
    plot_results(t, raw_spi, tri_spi, ema_spi, profile, title)


if __name__ == "__main__":
    main()
