import argparse
import json
import sys
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import numpy as np
except ModuleNotFoundError:
    raise SystemExit("matplotlib and numpy are required")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibration_math import (
    FORCE_KEYS,
    average_force,
    mean_emg,
    normalize_emg,
    ridge_fit,
    robust_emg_peak,
    sample_windows,
    slot_emg_values,
    spi_co,
    trim_samples,
)


def load_profile(path):
    with open(path, "r", encoding="utf-8-sig") as file:
        return json.load(file)


def dot(a, b):
    return sum(float(x) * float(y) for x, y in zip(a, b))


def pick_slots(values, slots, count):
    if not values:
        return []
    if slots:
        return [float(values[index]) for index in slots if index < len(values)]
    return [float(value) for value in values[:count]]


def repeat_sort_key(repeat):
    try:
        return 0, int(repeat)
    except (TypeError, ValueError):
        return 1, str(repeat)


def repeat_ids(raw_repeats):
    return sorted(
        {
            str(repeat)
            for repeat_set in raw_repeats.values()
            for repeat in repeat_set
        },
        key=repeat_sort_key,
    )


def repeat_emg_reference(raw_repeats, repeat):
    references = []
    for key in ("left_100", "right_100", "bracing"):
        reference = robust_emg_peak(
            trim_samples(raw_repeats.get(key, {}).get(repeat, []))
        )
        if reference:
            references.append(reference)
    if not references:
        return []
    width = min(len(reference) for reference in references)
    return np.max(
        np.asarray([reference[:width] for reference in references]), axis=0
    ).tolist()


def mean_normalized_u(samples, rest, ref, slots, count):
    rows = []
    for sample in samples:
        normalized = normalize_emg(slot_emg_values(sample), rest, ref)
        u = pick_slots(normalized, slots, count)
        if len(u) == count:
            rows.append(u)
    if not rows:
        return []
    return np.mean(np.asarray(rows, dtype=float), axis=0).tolist()


def fit_force_model(points, alpha):
    points = [point for point in points if point["label"] != "bracing"]
    if len(points) < 2:
        return None
    weights, bias = ridge_fit(
        [point["u"] for point in points],
        [point["force"] for point in points],
        alpha,
    )
    return weights.tolist(), bias


def build_window_points(raw_repeats, repeat, rest, ref, slots, count):
    points = []
    for key in FORCE_KEYS:
        samples = raw_repeats.get(key, {}).get(repeat, [])
        direction = -1.0 if key.startswith("left_") else 1.0
        for window in sample_windows(samples):
            u = mean_normalized_u(window, rest, ref, slots, count)
            force = direction * average_force(window, direction)
            if len(u) == count and force != 0.0:
                points.append(
                    {
                        "label": key,
                        "u": u,
                        "force": force,
                    }
                )

    samples = raw_repeats.get("emg_rest", {}).get(repeat, [])
    for window in sample_windows(samples):
        u = mean_normalized_u(window, rest, ref, slots, count)
        if len(u) == count:
            points.append(
                {
                    "label": "rest",
                    "u": u,
                    "force": 0.0,
                }
            )

    samples = trim_samples(raw_repeats.get("bracing", {}).get(repeat, []))
    if samples:
        bracing_peak = robust_emg_peak(samples)
        u = pick_slots(normalize_emg(bracing_peak, rest, ref), slots, count)
        if len(u) == count:
            points.append(
                {
                    "label": "bracing",
                    "u": u,
                    "force": 0.0,
                }
            )
    return points


def build_repeat_models(profile):
    raw_repeats = profile.get("raw_repeats", {})
    slots = profile.get("fit_slots", [])
    count = len(profile.get("emg_force_weights", []))
    alpha = float(profile.get("ridge_alpha", 0.1))
    common_rest = profile.get("emg_rest", [])
    common_ref = profile.get("emg_ref", []) or profile.get("emg_bracing", [])
    models = {}

    for repeat in repeat_ids(raw_repeats):
        rest_samples = raw_repeats.get("emg_rest", {}).get(repeat, [])
        rest = mean_emg(trim_samples(rest_samples))
        ref = repeat_emg_reference(raw_repeats, repeat)
        if not rest or not ref:
            print(f"Repeat {repeat}: missing EMG rest/reference; fit skipped.")
            continue

        fit_points = build_window_points(
            raw_repeats, repeat, rest, ref, slots, count
        )
        common_points = build_window_points(
            raw_repeats, repeat, common_rest, common_ref, slots, count
        )

        labels = {point["label"] for point in fit_points}
        missing = [
            key for key in (*FORCE_KEYS, "rest", "bracing") if key not in labels
        ]
        if missing:
            print(f"Repeat {repeat}: incomplete ({', '.join(missing)}); fit skipped.")
            continue

        fitted = fit_force_model(fit_points, alpha)
        if fitted is None:
            print(f"Repeat {repeat}: insufficient fit points; skipped.")
            continue
        weights, bias = fitted
        models[repeat] = {
            "weights": weights,
            "bias": bias,
            "points": fit_points,
            "common_points": common_points,
        }
        fit_count = sum(point["label"] != "bracing" for point in fit_points)
        print(
            f"Repeat {repeat}: {fit_count} window points, alpha={alpha:g}, "
            f"w={[round(value, 6) for value in weights]}, b={bias:.6f}"
        )
    return models


def average_fit_points(profile, count):
    grouped = {}
    for trial in profile.get("fit_trials", []):
        u = [float(value) for value in trial.get("emg_mean", [])]
        if len(u) != count:
            continue
        key = trial.get("key", "")
        grouped.setdefault(key, []).append((u, float(trial.get("force_mean", 0.0))))
    points = []
    for key in (*FORCE_KEYS, "rest", "bracing"):
        values = grouped.get(key, [])
        if not values:
            continue
        points.append(
            {
                "label": key,
                "u": np.mean([value[0] for value in values], axis=0).tolist(),
                "force": float(np.mean([value[1] for value in values])),
            }
        )
    return points


def fit_line(points):
    if len(points) < 2:
        return None
    x = np.asarray([point[0] for point in points], dtype=float)
    y = np.asarray([point[1] for point in points], dtype=float)
    if np.ptp(x) < 1e-12:
        return None
    slope, intercept = np.polyfit(x, y, 1)
    return float(slope), float(intercept)


def repeat_colors(repeats):
    colors = plt.get_cmap("tab10").colors
    return {
        repeat: colors[index % len(colors)] for index, repeat in enumerate(repeats)
    }


def plot_condition_fits(profile, profile_name, repeat_models):
    saved_weights = [float(value) for value in profile.get("emg_force_weights", [])]
    saved_bias = float(profile.get("emg_force_bias", 0.0))
    average_points = average_fit_points(profile, len(saved_weights))
    repeats = sorted(repeat_models, key=repeat_sort_key)
    colors = repeat_colors(repeats)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    ax = axes[0]
    for repeat in repeats:
        model = repeat_models[repeat]
        weights = model["weights"]
        bias = model["bias"]
        points = model["points"]
        x = [dot(weights, point["u"]) for point in points]
        y = [point["force"] for point in points]
        color = colors[repeat]
        ax.scatter(x, y, color=color, s=34, alpha=0.75, label=f"Repeat {repeat}")
        line_x = [min(x), max(x)]
        ax.plot(
            line_x,
            [value + bias for value in line_x],
            color=color,
            linewidth=1.8,
            label=f"Repeat {repeat} fit",
        )

    average_x = [dot(saved_weights, point["u"]) for point in average_points]
    average_y = [point["force"] for point in average_points]
    ax.scatter(
        average_x,
        average_y,
        facecolors="white",
        edgecolors="black",
        linewidths=1.5,
        s=70,
        label="All-repeat condition means",
        zorder=5,
    )
    if average_x:
        line_x = [min(average_x), max(average_x)]
        ax.plot(
            line_x,
            [value + saved_bias for value in line_x],
            color="black",
            linewidth=4.0,
            label="All-repeat fit",
            zorder=4,
        )
    for x, y, point in zip(average_x, average_y, average_points):
        ax.annotate(
            point["label"], (x, y), textcoords="offset points", xytext=(5, 5), fontsize=8
        )
    ax.axhline(0, color="0.75", linewidth=1)
    ax.axvline(0, color="0.75", linewidth=1)
    ax.set_xlabel("Weighted EMG term, sum(w_i u_i)")
    ax.set_ylabel("Mean interaction force Fx (N)")
    ax.set_title("Repeat and all-repeat EMG-force fits")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    for repeat in repeats:
        model = repeat_models[repeat]
        points = model["common_points"]
        force_values = [
            dot(saved_weights, point["u"]) + saved_bias for point in points
        ]
        spi_values = [
            spi_co(point["u"], saved_weights) for point in points
        ]
        ax.scatter(
            force_values,
            spi_values,
            color=colors[repeat],
            s=34,
            alpha=0.75,
            label=f"Repeat {repeat}",
        )

    average_force = [dot(saved_weights, point["u"]) + saved_bias for point in average_points]
    average_spi = [
        spi_co(point["u"], saved_weights) for point in average_points
    ]
    ax.scatter(
        average_force,
        average_spi,
        facecolors="white",
        edgecolors="black",
        linewidths=1.5,
        s=70,
        label="All-repeat means",
        zorder=5,
    )
    for x, y, point in zip(average_force, average_spi, average_points):
        ax.annotate(
            point["label"], (x, y), textcoords="offset points", xytext=(5, 5), fontsize=8
        )
    ax.axhline(0, color="0.75", linewidth=1)
    ax.axvline(0, color="0.75", linewidth=1)
    ax.set_xlabel("Force proxy, sum(w_i u_i) + b (N)")
    ax.set_ylabel("EMG co-contraction proxy")
    ax.set_title("Force proxy vs co-contraction (all-repeat model)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.suptitle(profile_name)
    fig.tight_layout()
    return fig


def build_raw_weighted_force_points(profile):
    raw_repeats = profile.get("raw_repeats", {})
    weights = [float(value) for value in profile.get("emg_force_weights", [])]
    rest = profile.get("emg_rest", [])
    ref = profile.get("emg_ref", []) or profile.get("emg_bracing", [])
    slots = profile.get("fit_slots", [])
    count = len(weights)
    points_by_repeat = {}

    for repeat in repeat_ids(raw_repeats):
        points = []
        counts = {}
        for key in FORCE_KEYS:
            repeat_set = raw_repeats.get(key, {})
            accepted = 0
            for sample in trim_samples(repeat_set.get(repeat, [])):
                force = sample.get("m2Force", [])
                if not force:
                    continue
                normalized = normalize_emg(slot_emg_values(sample), rest, ref)
                u = pick_slots(normalized, slots, count)
                if len(u) != count:
                    continue
                weighted_term = dot(weights, u)
                interaction_force = float(force[0])
                points.append((weighted_term, interaction_force))
                accepted += 1
            counts[key] = accepted
        points_by_repeat[repeat] = points
        count_text = ", ".join(f"{key}={count_value}" for key, count_value in counts.items())
        print(f"Repeat {repeat}: raw n={len(points)} ({count_text})")
    return points_by_repeat


def plot_raw_weighted_force_fits(profile, profile_name):
    points_by_repeat = build_raw_weighted_force_points(profile)
    if not points_by_repeat:
        print("No raw_repeats found; raw repeat fit plot skipped.")
        return None

    repeats = sorted(points_by_repeat, key=repeat_sort_key)
    colors = repeat_colors(repeats)
    all_points = []
    fig, ax = plt.subplots(figsize=(8.5, 6.5))

    for repeat in repeats:
        points = points_by_repeat[repeat]
        if not points:
            continue
        all_points.extend(points)
        color = colors[repeat]
        ax.scatter(
            [point[0] for point in points],
            [point[1] for point in points],
            s=14,
            alpha=0.22,
            color=color,
            edgecolors="none",
            label=f"Repeat {repeat} samples (n={len(points)})",
        )
        fitted = fit_line(points)
        if fitted is not None:
            slope, intercept = fitted
            x_range = [min(point[0] for point in points), max(point[0] for point in points)]
            ax.plot(
                x_range,
                [slope * value + intercept for value in x_range],
                color=color,
                linewidth=1.8,
                label=f"Repeat {repeat} fit",
            )

    fitted = fit_line(all_points)
    if fitted is not None:
        slope, intercept = fitted
        x_range = [min(point[0] for point in all_points), max(point[0] for point in all_points)]
        ax.plot(
            x_range,
            [slope * value + intercept for value in x_range],
            color="black",
            linewidth=4.0,
            label=f"All repeats fit (n={len(all_points)})",
            zorder=5,
        )

    ax.axhline(0, color="0.75", linewidth=1)
    ax.axvline(0, color="0.75", linewidth=1)
    ax.set_xlabel("Weighted EMG term, sum(w_i u_i)")
    ax.set_ylabel("Interaction force Fx (N)")
    ax.set_title(f"{profile_name}: raw weighted EMG term vs force")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    return fig


def pick_json_file():
    try:
        import tkinter as tk
        from tkinter import filedialog

        root = tk.Tk()
        root.withdraw()
        path = filedialog.askopenfilename(
            title="Select calibration JSON file, e.g. P01_calibration.json",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
        )
        root.destroy()
        return path or ""
    except Exception:
        return ""


def main():
    parser = argparse.ArgumentParser(
        description="Plot EMG-force calibration fits from a saved calibration JSON."
    )
    parser.add_argument(
        "json_path", nargs="?", default="", help="Calibration JSON file."
    )
    parser.add_argument("--out", default="", help="Optional output image path.")
    args = parser.parse_args()

    json_path = args.json_path or pick_json_file()
    if not json_path:
        raise SystemExit("No calibration JSON file selected.")

    path = Path(json_path)
    profile = load_profile(path)
    saved_weights = profile.get("emg_force_weights", [])
    if not saved_weights or not profile.get("fit_trials", []):
        raise SystemExit("No saved EMG-force fit found. Re-run calibration and save.")

    repeat_models = build_repeat_models(profile)
    fit_fig = plot_condition_fits(profile, path.name, repeat_models)
    raw_fig = plot_raw_weighted_force_fits(profile, path.name)

    if args.out:
        out = Path(args.out)
        fit_fig.savefig(out, dpi=200, bbox_inches="tight")
        print(f"Saved: {out}")
        if raw_fig is not None:
            suffix = out.suffix or ".png"
            repeat_out = out.with_name(f"{out.stem}_repeats{suffix}")
            raw_fig.savefig(repeat_out, dpi=200, bbox_inches="tight")
            print(f"Saved: {repeat_out}")

    saved_bias = float(profile.get("emg_force_bias", 0.0))
    print(
        f"All-repeat saved w: {[round(float(weight), 6) for weight in saved_weights]}, "
        f"b={saved_bias:.6f}"
    )
    plt.show()


if __name__ == "__main__":
    main()
