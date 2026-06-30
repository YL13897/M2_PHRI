import argparse
import json
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ModuleNotFoundError:
    raise SystemExit("matplotlib is required: python -m pip install matplotlib")


def load_profile(path):
    with open(path, "r", encoding="utf-8-sig") as f:
        return json.load(f)


def dot(a, b):
    return sum(float(x) * float(y) for x, y in zip(a, b))


def norm_emg(emg, rest, ref):
    out = []
    for e, r, m in zip(emg or [], rest or [], ref or []):
        denom = max(float(m) - float(r), 1e-6)
        out.append(max(0.0, min(1.0, (float(e) - float(r)) / denom)))
    return out


def pick_slots(values, slots, count):
    if not values:
        return []
    if slots:
        return [float(values[i]) for i in slots if i < len(values)]
    return [float(v) for v in values[:count]]


def force_proxy(weights, bias, u):
    return dot(weights, u) + bias


def spi_proxy(weights, bias, u):
    return dot([abs(float(w)) for w in weights], u) + bias


def solve_linear(a, b):
    n = len(b)
    m = [row[:] + [float(bi)] for row, bi in zip(a, b)]
    for i in range(n):
        pivot = max(range(i, n), key=lambda r: abs(m[r][i]))
        if abs(m[pivot][i]) < 1e-12:
            return []
        m[i], m[pivot] = m[pivot], m[i]
        scale = m[i][i]
        for c in range(i, n + 1):
            m[i][c] /= scale
        for r in range(n):
            if r == i:
                continue
            factor = m[r][i]
            for c in range(i, n + 1):
                m[r][c] -= factor * m[i][c]
    return [m[i][n] for i in range(n)]


def refit_force(points, count):
    xtx = [[0.0 for _ in range(count + 1)] for _ in range(count + 1)]
    xty = [0.0 for _ in range(count + 1)]
    for point in points:
        row = [float(v) for v in point["u"][:count]] + [1.0]
        y = float(point["force"])
        for r in range(count + 1):
            xty[r] += row[r] * y
            for c in range(count + 1):
                xtx[r][c] += row[r] * row[c]

    coeffs = solve_linear(xtx, xty)
    if not coeffs:
        return [], 0.0
    return coeffs[:-1], coeffs[-1]


def pick_json_file():
    try:
        import tkinter as tk
        from tkinter import filedialog

        root = tk.Tk()
        root.withdraw()
        path = filedialog.askopenfilename(
            title="Select calibration JSON file, e.g. P01_calibration.json (not EmgLogs/EmgDebug CSV)",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
        )
        root.destroy()
        return path or ""
    except Exception:
        return ""


def main():
    parser = argparse.ArgumentParser(
        description="Plot EMG-force calibration fit from saved calibration JSON, not EMG/debug CSV."
    )
    parser.add_argument("json_path", nargs="?", default="", help="Calibration JSON file, e.g. P01_calibration.json.")
    parser.add_argument("--out", default="", help="Optional output image path.")
    args = parser.parse_args()

    json_path = args.json_path or pick_json_file()
    if not json_path:
        raise SystemExit("No calibration JSON file selected. Select a saved *_calibration.json file, not EmgLogs/EmgDebug CSV.")

    path = Path(json_path)
    profile = load_profile(path)
    trials = profile.get("fit_trials", [])
    saved_weights = profile.get("emg_force_weights", [])
    saved_bias = float(profile.get("emg_force_bias", 0.0))
    slots = profile.get("fit_slots", [])

    force_trials = [trial for trial in trials if trial.get("key", "") not in ("rest", "bracing")]
    count = len(saved_weights) if saved_weights else len(force_trials[0].get("emg_mean", [])) if force_trials else 0
    if not force_trials or count <= 0:
        raise SystemExit("No usable force fit trials found. Re-run calibration and save the profile.")

    rest = profile.get("emg_rest", [])
    ref = profile.get("emg_ref", []) or profile.get("emg_bracing", [])
    force_points = [
        {"label": trial.get("key", ""), "u": trial.get("emg_mean", []), "force": float(trial["force_mean"]), "extra": False}
        for trial in force_trials
    ]
    points = list(force_points)
    for label, emg in (("rest", rest), ("bracing", profile.get("emg_bracing", []))):
        u = pick_slots(norm_emg(emg, rest, ref), slots, count)
        if len(u) == count:
            points.append({"label": label, "u": u, "force": 0.0, "extra": True})

    old_weights, old_bias = refit_force(force_points, count)
    new_weights, new_bias = refit_force(points, count)
    if not old_weights or not new_weights:
        raise SystemExit("Least-squares fit failed. Check calibration trial data.")

    x = [dot(old_weights, point["u"]) for point in force_points]
    y = [point["force"] for point in force_points]
    labels = [point["label"] for point in force_points]
    x_line = sorted(x)
    y_line = [xi + old_bias for xi in x_line]

    fig, axes = plt.subplots(1, 3, figsize=(16, 4.8))

    ax = axes[0]
    ax.scatter(x, y, color="#1f77b4", label="3s trial mean")
    ax.plot(x_line, y_line, color="#d62728", label=f"old fit: + {old_bias:.3f}")
    for xi, yi, label in zip(x, y, labels):
        ax.annotate(label, (xi, yi), textcoords="offset points", xytext=(5, 5), fontsize=8)
    ax.axhline(0, color="0.75", linewidth=1)
    ax.axvline(0, color="0.75", linewidth=1)
    ax.set_xlabel("Weighted EMG term, sum(w_i u_i)")
    ax.set_ylabel("Mean interaction force Fx (N)")
    ax.set_title("EMG-force fit")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    fx = []
    spi = []
    for point in force_points:
        u = point["u"]
        fx.append(force_proxy(new_weights, new_bias, u))
        spi.append(spi_proxy(new_weights, new_bias, u))

    ax.scatter(fx, spi, color="#1f77b4", label="force trials")
    for xi, yi, label in zip(fx, spi, labels):
        ax.annotate(label, (xi, yi), textcoords="offset points", xytext=(5, 5), fontsize=8)

    extra = [
        ("rest", rest),
        ("bracing", profile.get("emg_bracing", [])),
    ]
    for label, emg in extra:
        u = pick_slots(norm_emg(emg, rest, ref), slots, count)
        if len(u) != count:
            continue
        xi = force_proxy(new_weights, new_bias, u)
        yi = spi_proxy(new_weights, new_bias, u)
        ax.scatter([xi], [yi], marker="x", s=80, color="#d62728")
        ax.annotate(label, (xi, yi), textcoords="offset points", xytext=(5, 5), fontsize=8)

    ax.axhline(0, color="0.75", linewidth=1)
    ax.axvline(0, color="0.75", linewidth=1)
    ax.set_xlabel("Force proxy, sum(w_i u_i) + b (N)")
    ax.set_ylabel("Co-contraction proxy, sum(|w_i| u_i) + b")
    ax.set_title("Force proxy vs co-contraction")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    old_x = [dot(old_weights, point["u"]) for point in points]
    new_x = [dot(new_weights, point["u"]) for point in points]
    yy = [point["force"] for point in points]
    point_labels = [point["label"] for point in points]
    colors = ["#d62728" if point["extra"] else "#1f77b4" for point in points]
    markers = ["x" if point["extra"] else "o" for point in points]

    for xi, yi, label, color, marker, point in zip(old_x, yy, point_labels, colors, markers, points):
        text = f"{label} old" if point["extra"] else label
        ax.scatter([xi], [yi], color=color, marker=marker, s=70 if marker == "x" else 35)
        ax.annotate(text, (xi, yi), textcoords="offset points", xytext=(5, 5), fontsize=8)

    old_line_x = sorted(old_x)
    ax.plot(old_line_x, [xi + old_bias for xi in old_line_x], color="#d62728", label=f"old fit: + {old_bias:.3f}")

    for xi, yi, point in zip(new_x, yy, points):
        if point["extra"]:
            ax.scatter([xi], [yi], color="#2ca02c", marker="x", s=90)
            ax.annotate(f"{point['label']} new", (xi, yi), textcoords="offset points", xytext=(5, -12), fontsize=8)
        else:
            ax.scatter([xi], [yi], edgecolors="#2ca02c", facecolors="none", marker="o", s=45)
    new_line_x = sorted(new_x)
    ax.plot(new_line_x, [xi + new_bias for xi in new_line_x], color="#2ca02c", linestyle="--", label=f"with rest/bracing: + {new_bias:.3f}")

    ax.axhline(0, color="0.75", linewidth=1)
    ax.axvline(0, color="0.75", linewidth=1)
    ax.set_xlabel("Weighted EMG term, sum(w_i u_i)")
    ax.set_ylabel("Mean interaction force Fx (N)")
    ax.set_title("Fit comparison")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.suptitle(path.name)
    fig.tight_layout()

    if args.out:
        out = Path(args.out)
        fig.savefig(out, dpi=200)
        print(f"Saved: {out}")
    if saved_weights:
        print(f"Saved w: {[round(float(w), 6) for w in saved_weights]}, b: {saved_bias:.6f}")
    print(f"Old fit w: {[round(float(w), 6) for w in old_weights]}, b: {old_bias:.6f}")
    print(f"With rest/bracing w: {[round(float(w), 6) for w in new_weights]}, b: {new_bias:.6f}")
    plt.show()


if __name__ == "__main__":
    main()
