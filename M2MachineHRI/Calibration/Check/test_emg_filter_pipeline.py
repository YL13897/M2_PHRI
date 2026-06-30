import csv
import math
from pathlib import Path
from tkinter import Tk, filedialog

import matplotlib.pyplot as plt
import numpy as np

# ----------------------------- Parameters ------------------------------------

# Pipeline:
# raw -> band-pass -> rectified+envelope OR RMS -> optional median
#     -> downsample -> optional triangular filter -> EMA
DATA_DIR = r"D:\yixianglin\Desktop\PHRI_Data"
MAX_CHANNELS = 2
HIGH_PASS_HZ = 20.0
LOW_PASS_HZ = 450.0
USE_RMS = 0
RMS_WINDOW_MS = 100.0
ENVELOPE_HZ = 10
ENVELOPE_ORDER = 2
USE_MEDIAN_FILTER = 0
MEDIAN_FILTER_MS = 50.0
DOWNSAMPLE_HZ = 100.0
USE_TRIANGULAR_FILTER = 1
TRIANGULAR_FILTER_MS = 100.0
EMA_TAU_MS = 200
PLOT_START_SEC = 0.0
PLOT_DURATION_SEC = 15.0

# Matches Unity EMGFilter: two cascaded biquads with Q = 1 / sqrt(2).
BUTTERWORTH_Q = 1.0 / math.sqrt(2.0)


def choose_csv():
    root = Tk()
    root.withdraw()
    path = filedialog.askopenfilename(
        title="Select EmgDebug CSV",
        initialdir=DATA_DIR,
        filetypes=[("EmgDebug CSV", "EmgDebug_*.csv"), ("CSV files", "*.csv")],
    )
    root.destroy()
    return Path(path) if path else None


def channel_number(name):
    return int(name.rsplit("ch", 1)[1])


def read_raw_emg(path):
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        channels = sorted(
            [name for name in fields if name.strip().lower().startswith("raw_ch")],
            key=channel_number,
        )[:MAX_CHANNELS]

        if "emg_time" not in fields:
            raise ValueError(f"No emg_time column in {path}.")
        if not channels:
            raise ValueError(f"No raw_ch* columns in {path}.")

        times = []
        values = {name: [] for name in channels}
        for row in reader:
            try:
                t = float(row["emg_time"])
                sample = [float(row[name]) for name in channels]
            except (KeyError, TypeError, ValueError):
                continue
            times.append(t)
            for name, value in zip(channels, sample):
                values[name].append(value)

    if len(times) < 4:
        raise ValueError(f"Not enough valid raw EMG samples in {path}.")

    t = np.asarray(times, dtype=float)
    order = np.argsort(t)
    return t[order], {
        name: np.asarray(x, dtype=float)[order] for name, x in values.items()
    }


def biquad_coefficients(kind, sample_rate, cutoff, q):
    omega = 2.0 * math.pi * cutoff / sample_rate
    cos_omega = math.cos(omega)
    alpha = math.sin(omega) / (2.0 * q)

    if kind == "lowpass":
        b0 = (1.0 - cos_omega) * 0.5
        b1 = 1.0 - cos_omega
        b2 = b0
    else:
        b0 = (1.0 + cos_omega) * 0.5
        b1 = -(1.0 + cos_omega)
        b2 = b0

    a0 = 1.0 + alpha
    return b0 / a0, b1 / a0, b2 / a0, -2.0 * cos_omega / a0, (1.0 - alpha) / a0


def apply_biquad(x, coeffs):
    b0, b1, b2, a1, a2 = coeffs
    y = np.empty_like(x)
    z1 = 0.0
    z2 = 0.0
    for i, value in enumerate(x):
        output = b0 * value + z1
        z1 = b1 * value - a1 * output + z2
        z2 = b2 * value - a2 * output
        y[i] = output
    return y


def ema_filter(x, sample_rate, tau_ms):
    tau = max(tau_ms / 1000.0, 1e-6)
    alpha = 1.0 - math.exp(-1.0 / (sample_rate * tau))
    y = np.empty_like(x)
    state = 0.0
    for i, value in enumerate(x):
        state += alpha * (value - state)
        y[i] = state
    return y


def median_filter(x, sample_rate, window_ms):
    window = max(1, round(sample_rate * window_ms / 1000.0))
    if window % 2 == 0:
        window += 1

    half = window // 2
    y = np.empty_like(x)
    for i in range(len(x)):
        start = max(0, i - half)
        end = min(len(x), i + half + 1)
        y[i] = np.median(x[start:end])
    return y


def rms_filter(x, sample_rate, window_ms):
    window = max(1, round(sample_rate * window_ms / 1000.0))
    y = np.empty_like(x)
    sum_sq = 0.0
    for i, value in enumerate(x):
        sum_sq += value * value
        if i >= window:
            old = x[i - window]
            sum_sq -= old * old
        count = min(i + 1, window)
        y[i] = math.sqrt(max(sum_sq / count, 0.0))
    return y


def triangular_filter(x, sample_rate, window_ms):
    window = max(1, round(sample_rate * window_ms / 1000.0))
    if window % 2 == 0:
        window += 1

    half = window // 2
    weights = np.arange(1, half + 2, dtype=float)
    weights = np.concatenate((weights, weights[-2::-1]))
    weights /= np.sum(weights)

    y = np.empty_like(x)
    for i in range(len(x)):
        start = max(0, i - half)
        end = min(len(x), i + half + 1)
        w_start = half - (i - start)
        w_end = w_start + (end - start)
        w = weights[w_start:w_end]
        y[i] = np.sum(x[start:end] * w) / np.sum(w)
    return y


def downsample(t, x, target_hz):
    target_hz = max(target_hz, 1e-6)
    start = t[0]
    end = t[-1]
    target_t = np.arange(start, end + 0.5 / target_hz, 1.0 / target_hz)
    target_x = np.interp(target_t, t, x)
    return target_t, target_x


def unity_pipeline(t, raw, sample_rate):
    nyquist = 0.5 * sample_rate
    hp = min(max(HIGH_PASS_HZ, 0.001), nyquist - 0.001)
    lp = min(max(LOW_PASS_HZ, hp + 0.001), nyquist - 0.001)

    highpass = biquad_coefficients("highpass", sample_rate, hp, BUTTERWORTH_Q)
    lowpass = biquad_coefficients("lowpass", sample_rate, lp, BUTTERWORTH_Q)

    filtered = apply_biquad(raw, highpass)
    filtered = apply_biquad(filtered, highpass)
    filtered = apply_biquad(filtered, lowpass)
    filtered = apply_biquad(filtered, lowpass)
    rectified = np.abs(filtered)

    if USE_RMS:
        envelope = rms_filter(filtered, sample_rate, RMS_WINDOW_MS)
    elif ENVELOPE_ORDER == 1:
        dt = 1.0 / sample_rate
        rc = 1.0 / (2.0 * math.pi * ENVELOPE_HZ)
        alpha = dt / (rc + dt)
        envelope = np.empty_like(rectified)
        state = 0.0
        for i, value in enumerate(rectified):
            state += alpha * (value - state)
            envelope[i] = state
    elif ENVELOPE_ORDER == 2:
        envelope_filter = biquad_coefficients(
            "lowpass", sample_rate, ENVELOPE_HZ, BUTTERWORTH_Q
        )
        envelope = apply_biquad(rectified, envelope_filter)
    else:
        raise ValueError("ENVELOPE_ORDER must be 1 or 2.")

    median = (
        median_filter(envelope, sample_rate, MEDIAN_FILTER_MS)
        if USE_MEDIAN_FILTER
        else envelope
    )
    ds_t, ds_signal = downsample(t, median, DOWNSAMPLE_HZ)
    triangular = (
        triangular_filter(ds_signal, DOWNSAMPLE_HZ, TRIANGULAR_FILTER_MS)
        if USE_TRIANGULAR_FILTER
        else ds_signal
    )
    smooth = ema_filter(triangular, DOWNSAMPLE_HZ, EMA_TAU_MS)

    return filtered, rectified, envelope, ds_t, triangular, smooth


def plot_pipeline(path, t, channels, results, sample_rate):
    start = t[0] + PLOT_START_SEC
    end = t[-1] if PLOT_DURATION_SEC <= 0 else min(t[-1], start + PLOT_DURATION_SEC)
    mask = (t >= start) & (t <= end)
    plot_t = t[mask] - start

    fig, axes = plt.subplots(
        len(channels),
        4,
        figsize=(17, 4 * len(channels)),
        squeeze=False,
        constrained_layout=True,
    )
    fig.suptitle(
        f"{path.name} | fs={sample_rate:.2f} Hz | HP={HIGH_PASS_HZ:g}, LP={LOW_PASS_HZ:g}, "
        f"{'RMS=' + format(RMS_WINDOW_MS, 'g') + ' ms' if USE_RMS else 'Env=' + format(ENVELOPE_HZ, 'g') + ' Hz (order=' + str(ENVELOPE_ORDER) + ')'}, "
        f"Median={'on' if USE_MEDIAN_FILTER else 'off'} ({MEDIAN_FILTER_MS:g} ms), "
        f"Downsample={DOWNSAMPLE_HZ:g} Hz, "
        f"Triangular={'on' if USE_TRIANGULAR_FILTER else 'off'} ({TRIANGULAR_FILTER_MS:g} ms), "
        f"EMA tau={EMA_TAU_MS:g} ms"
    )

    for row, (name, raw) in enumerate(channels.items()):
        filtered, rectified, envelope, ds_t, triangular, smooth = results[name]
        ds_mask = (ds_t >= start) & (ds_t <= end)
        ds_plot_t = ds_t[ds_mask] - start
        source_name = "RMS" if USE_RMS else "Envelope"
        smooth_name = source_name
        if USE_MEDIAN_FILTER:
            smooth_name += " + median"
        smooth_name += " + downsample"
        if USE_TRIANGULAR_FILTER:
            smooth_name += " + triangular"
        smooth_name += " + EMA"
        plots = (
            (plot_t, raw[mask], "Raw"),
            (plot_t, filtered[mask], "Band-pass filtered"),
            (plot_t, envelope[mask], source_name),
            (ds_plot_t, smooth[ds_mask], smooth_name),
        )
        y_min = min(np.min(envelope[mask]), np.min(smooth[ds_mask]))
        y_max = max(np.max(envelope[mask]), np.max(smooth[ds_mask]))
        y_pad = 0.05 * max(y_max - y_min, 1e-21)
        for col, (x, y, title) in enumerate(plots):
            axes[row, col].plot(x, y, linewidth=0.8)
            axes[row, col].set_title(f"{name}: {title}")
            axes[row, col].set_xlabel("Time (s)")
            if col >= 2:
                axes[row, col].set_ylim(y_min - y_pad, y_max + y_pad)
            axes[row, col].grid(True, alpha=0.3)

    fig2, axes2 = plt.subplots(
        len(channels),
        1,
        figsize=(12, 3.5 * len(channels)),
        squeeze=False,
        constrained_layout=True,
    )
    fig2.suptitle("Raw vs smoothed envelope/RMS (normalised)")

    for row, (name, raw) in enumerate(channels.items()):
        _, _, _, ds_t, _, smooth = results[name]
        ds_mask = (ds_t >= start) & (ds_t <= end)
        ds_plot_t = ds_t[ds_mask] - start
        raw_plot = raw[mask]
        smooth_plot = smooth[ds_mask]
        raw_scale = max(np.max(np.abs(raw_plot)), 1e-21)
        smooth_scale = max(np.max(np.abs(smooth_plot)), 1e-21)

        ax = axes2[row, 0]
        ax.plot(plot_t, raw_plot / raw_scale, linewidth=0.6, label="raw")
        ax.plot(ds_plot_t, smooth_plot / smooth_scale, linewidth=1.2, label="smoothed")
        ax.set_title(name)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Normalised amplitude")
        ax.grid(True, alpha=0.3)
        ax.legend()

    plt.show()


def main():
    path = choose_csv()
    if not path:
        return

    t, channels = read_raw_emg(path)
    dt = np.diff(t)
    dt = dt[dt > 0]
    if len(dt) == 0:
        raise ValueError("EMG time is not increasing.")
    sample_rate = 1.0 / float(np.median(dt))

    results = {
        name: unity_pipeline(t, raw, sample_rate) for name, raw in channels.items()
    }
    print(f"Loaded: {path}")
    print(f"Channels: {', '.join(channels)}")
    print(f"Samples: {len(t)}, fs~{sample_rate:.3f} Hz")
    plot_pipeline(path, t, channels, results, sample_rate)


if __name__ == "__main__":
    main()
