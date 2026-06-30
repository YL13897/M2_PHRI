import csv
import math
from pathlib import Path
from tkinter import Tk, filedialog

import matplotlib.pyplot as plt
import numpy as np

# Load and analyze force sensor data and EMG envelopes from CSV logs, then plot the time series and their FFT spectra.


def choose_csv(title, pattern):
    root = Tk()
    root.withdraw()
    path = filedialog.askopenfilename(
        title=title,
        initialdir=r"D:\yixianglin\Desktop\PHRI_Data",
        filetypes=[
            ("Expected CSV", pattern),
            ("CSV files", "*.csv"),
            ("All files", "*.*"),
        ],
    )
    root.destroy()
    return Path(path) if path else None


def read_force_csv(path):
    t = []
    fx = []
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for row in reader:
            time_text = (row.get("unity_time") or "").strip()
            force_text = (row.get("force_sensor_x") or "").strip()
            if not time_text or not force_text:
                continue
            try:
                t.append(float(time_text))
                fx.append(float(force_text))
            except ValueError:
                continue

    if len(t) < 4:
        raise ValueError("Not enough valid unity_time / force_sensor_x samples.")

    t = np.asarray(t, dtype=float)
    fx = np.asarray(fx, dtype=float)
    order = np.argsort(t)
    return t[order], fx[order]


def read_emg_csv(path):
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        channels = [
            name
            for name in fieldnames
            if name.strip().lower().startswith("envelope_ch")
        ]
        channels.sort(key=lambda name: int(name.rsplit("ch", 1)[1]))
        channels = channels[:6]
        samples = {name: ([], []) for name in channels}

        for row in reader:
            time_text = (row.get("global_t") or "").strip()
            if not time_text:
                continue
            try:
                time_value = float(time_text)
            except ValueError:
                continue

            for name in channels:
                value_text = (row.get(name) or "").strip()
                try:
                    value = float(value_text)
                except ValueError:
                    continue
                samples[name][0].append(time_value)
                samples[name][1].append(value)

    valid = {}
    for name, (t, x) in samples.items():
        if len(t) >= 4:
            valid[name] = (np.asarray(t, dtype=float), np.asarray(x, dtype=float))
    if not valid:
        raise ValueError(f"No valid envelope_ch* data in {path}. Columns: {fieldnames}")
    return valid


def read_debug_emg_csv(path):
    names = [
        "raw_ch1",
        "filtered_ch1",
        "envelope_ch1",
        "raw_ch2",
        "filtered_ch2",
        "envelope_ch2",
    ]
    t = []
    samples = {name: [] for name in names}

    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        missing = [
            name
            for name in ["emg_time", *names]
            if name not in (reader.fieldnames or [])
        ]
        if missing:
            raise ValueError(f"Missing columns in {path}: {missing}")

        for row in reader:
            try:
                time_value = float(row["emg_time"])
                values = [float(row[name]) for name in names]
            except (TypeError, ValueError):
                continue
            t.append(time_value)
            for name, value in zip(names, values):
                samples[name].append(value)

    if len(t) < 4:
        raise ValueError(f"Not enough valid EmgDebug samples in {path}.")

    t = np.asarray(t, dtype=float)
    return {
        name: (t, np.asarray(values, dtype=float)) for name, values in samples.items()
    }


def uniform_resample(t, x):
    t = t - t[0]
    dt = np.diff(t)
    dt = dt[dt > 0]
    if len(dt) == 0:
        raise ValueError("unity_time is not increasing.")

    sample_dt = float(np.median(dt))
    fs = 1.0 / sample_dt
    n = int(math.floor((t[-1] - t[0]) / sample_dt)) + 1
    tu = np.arange(n, dtype=float) * sample_dt
    xu = np.interp(tu, t, x)
    return tu, xu, fs


def spectrum(t, x):
    tu, xu, fs = uniform_resample(t, x)
    xu = xu - np.mean(xu)
    window = np.hanning(len(xu))
    xw = xu * window

    freq = np.fft.rfftfreq(len(xw), d=1.0 / fs)
    amp = 2.0 * np.abs(np.fft.rfft(xw)) / max(np.sum(window), 1e-12)
    return tu, xu, fs, freq, amp


def plot_force(path, t, fx, tu, xu, fs, freq, amp):
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
    fig.suptitle(path.name)

    axes[0].plot(t - t[0], fx, alpha=0.45, label="raw logged")
    axes[0].plot(tu, xu, linewidth=1.2, label=f"uniform resample, fs={fs:.2f} Hz")
    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Force sensor X (N)")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(freq, amp)
    axes[1].set_xlim(0, fs / 2.0)
    axes[1].set_xlabel("Frequency (Hz)")
    axes[1].set_ylabel("Amplitude (N)")
    axes[1].grid(True, alpha=0.3)

    if len(freq) > 1:
        peak_idx = int(np.argmax(amp[1:]) + 1)
        axes[1].axvline(freq[peak_idx], color="r", linestyle="--", alpha=0.6)
        axes[1].set_title(f"FFT amplitude spectrum, peak={freq[peak_idx]:.3f} Hz")


def plot_emg(path, samples):
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
    fig.suptitle(path.name)

    for name, (t, x) in samples.items():
        tu, xu, fs, freq, amp = spectrum(t, x)
        axes[0].plot(tu, xu, linewidth=0.9, label=name)
        axes[1].plot(freq, amp, linewidth=1.0, label=f"{name}, fs={fs:.1f} Hz")

    axes[0].set_xlabel("Time (s)")
    axes[0].set_ylabel("Demeaned envelope")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].set_xlabel("Frequency (Hz)")
    axes[1].set_ylabel("Amplitude")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()


def plot_debug_emg(path, samples):
    fig, axes = plt.subplots(2, 3, figsize=(15, 8), constrained_layout=True)
    fig.suptitle(path.name)

    for row, channel in enumerate((1, 2)):
        for col, signal in enumerate(("raw", "filtered", "envelope")):
            name = f"{signal}_ch{channel}"
            t, x = samples[name]
            _, _, fs, freq, amp = spectrum(t, x)
            ax = axes[row, col]
            ax.plot(freq, amp, linewidth=0.9)
            ax.set_xlim(0, min(60.0, fs / 2.0) if signal == "envelope" else fs / 2.0)
            ax.set_title(f"{name}, fs={fs:.1f} Hz")
            ax.set_xlabel("Frequency (Hz)")
            ax.set_ylabel("Amplitude")
            ax.grid(True, alpha=0.3)
            if signal == "envelope":
                ax.axvline(15.0, color="r", linestyle="--", alpha=0.6, label="15 Hz")
                ax.legend()


def main():
    force_path = choose_csv(
        "Select InteractionEstimatorDebug CSV", "InteractionEstimatorDebug_*.csv"
    )
    if not force_path:
        return
    emg_path = choose_csv("Select EmgLogs CSV", "EmgLogs_*.csv")
    if not emg_path:
        return
    debug_emg_path = choose_csv("Select EmgDebug CSV", "EmgDebug_*.csv")
    if not debug_emg_path:
        return

    t, fx = read_force_csv(force_path)
    tu, xu, fs, freq, amp = spectrum(t, fx)
    emg = read_emg_csv(emg_path)
    debug_emg = read_debug_emg_csv(debug_emg_path)

    print(f"Force: {force_path}")
    print(f"Force samples: raw={len(t)}, uniform={len(tu)}, fs~{fs:.3f} Hz")
    print(f"EMG: {emg_path}")
    print(f"EMG channels: {', '.join(emg)}")
    print(f"EMG debug: {debug_emg_path}")

    plot_force(force_path, t, fx, tu, xu, fs, freq, amp)
    plot_emg(emg_path, emg)
    plot_debug_emg(debug_emg_path, debug_emg)
    plt.show()


if __name__ == "__main__":
    main()
