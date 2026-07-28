import numpy as np

# -------------------------------------------------------------------------------
# calibration_math.py contains functions for processing raw calibration data,
# including normalization, fitting, and computing reference values.
# -------------------------------------------------------------------------------


MAX_EMG_CHANNELS = 10
FIT_DISCARD_SEC = 0.5
FIT_WINDOW_SEC = 0.5
FIT_DURATION_SEC = 3.0
RIDGE_ALPHAS = (1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0, 100.0)
DEFAULT_RIDGE_ALPHA = 0.1
FORCE_KEYS = (
    "left_30",
    "left_50",
    "left_70",
    "left_100",
    "right_30",
    "right_50",
    "right_70",
    "right_100",
)
CALIBRATION_LABELS = {
    "emg_rest": "EMG Rest",
    "left_30": "Left 30%",
    "left_50": "Left 50%",
    "left_70": "Left 70%",
    "left_100": "Left MVC",
    "right_30": "Right 30%",
    "right_50": "Right 50%",
    "right_70": "Right 70%",
    "right_100": "Right MVC",
    "bracing": "Hold Stiff MVC",
}


def clamp(value, low, high):
    return max(low, min(high, value))


def force_value(sample):
    force = sample.get("m2Force", []) if sample else []
    if len(force) < 1:
        return 0.0
    return float(force[0])


def emg_values(sample):
    emg = sample.get("emg", []) if sample else []
    return [float(v) for v in emg]


def slot_emg_values(sample):
    emg = emg_values(sample)
    channels = sample.get("emgChannels", []) if sample else []
    slots = sample.get("emgSlots", []) if sample else []
    values = []
    for channel in slots[:MAX_EMG_CHANNELS]:
        if channel <= 0:
            values.append(0.0)
            continue
        try:
            index = channels.index(channel)
        except ValueError:
            values.append(0.0)
            continue
        values.append(float(emg[index]) if index < len(emg) else 0.0)
    return values


def selected_emg_slots(sample):
    slots = sample.get("emgSlots", []) if sample else []
    mask_value = sample.get("spiSlotMask") if sample else None
    mask = int(mask_value) if mask_value is not None else (1 << MAX_EMG_CHANNELS) - 1
    return [
        i
        for i, channel in enumerate(slots[:MAX_EMG_CHANNELS])
        if channel > 0 and mask & (1 << i)
    ]


def _sample_time(sample):
    if "unityTicksUtc" in sample:
        return float(sample["unityTicksUtc"]) / 1e7
    if "m2Time" in sample:
        return float(sample["m2Time"])
    return None


def trim_samples(samples, discard=FIT_DISCARD_SEC, duration=FIT_DURATION_SEC):
    timed = [(sample, _sample_time(sample)) for sample in samples]
    timed = [(sample, time) for sample, time in timed if time is not None]
    if not timed:
        return []
    start = timed[0][1] + discard
    end = start + duration
    return [sample for sample, time in timed if start <= time < end]


def sample_windows(samples, window=FIT_WINDOW_SEC):
    if not samples:
        return []
    first_time = _sample_time(samples[0])
    if first_time is None:
        return []

    start = first_time + FIT_DISCARD_SEC
    windows = [[] for _ in range(int(round(FIT_DURATION_SEC / window)))]
    for sample in samples:
        time = _sample_time(sample)
        if time is None:
            continue
        elapsed = time - start
        if elapsed < 0:
            continue
        index = int(elapsed / window)
        if 0 <= index < len(windows):
            windows[index].append(sample)
    # Exclude sparse or truncated bins when loading interrupted/legacy recordings.
    return [
        samples
        for index, samples in enumerate(windows)
        if samples
        and _sample_time(samples[0]) <= start + (index + 0.2) * window
        and _sample_time(samples[-1]) >= start + (index + 0.8) * window
    ]


def mean_emg(samples):
    rows = []
    for sample in samples:
        row = slot_emg_values(sample)
        if row:
            rows.append(row)
    if not rows:
        return []
    return np.mean(np.asarray(rows, dtype=float), axis=0).tolist()


# Robust peak estimation using the 95th percentile to mitigate outliers.
def robust_emg_peak(samples, q=95):
    rows = []
    for sample in samples:
        row = slot_emg_values(sample)
        if row:
            rows.append(row)
    if not rows:
        return []
    return np.percentile(np.asarray(rows, dtype=float), q, axis=0).tolist()


def peak_force(samples, direction):
    values = []
    for sample in samples:
        fx = force_value(sample)
        if direction < 0 and fx < 0:
            values.append(abs(fx))
        if direction > 0 and fx > 0:
            values.append(abs(fx))
    return float(max(values)) if values else 0.0


def compute_emg_ref(profile):
    refs = [profile.emg_left_mvc, profile.emg_right_mvc, profile.emg_bracing]
    refs = [np.asarray(v, dtype=float) for v in refs if v]
    if not refs:
        return []
    return np.max(np.vstack(refs), axis=0).tolist()


def normalize_emg(emg, rest, ref):
    if not emg or not rest or not ref:
        return []
    emg_arr = np.asarray(emg, dtype=float)
    rest_arr = np.asarray(rest, dtype=float)
    ref_arr = np.asarray(ref, dtype=float)
    denom = np.maximum(ref_arr - rest_arr, 1e-6)
    out = (emg_arr - rest_arr) / denom
    return np.clip(out, 0.0, 1.0).tolist()


def spi_co(u, weights):
    p = 0.0
    n = 0.0
    for ui, wi in zip(u, weights):
        aw = abs(float(wi))
        if wi > 0:
            p += aw * float(ui)
        elif wi < 0:
            n += aw * float(ui)
    return p + n - abs(p - n)


def average_force(samples, direction):
    values = []
    for sample in samples:
        fx = force_value(sample)
        if direction < 0 and fx < 0:
            values.append(abs(fx))
        if direction > 0 and fx > 0:
            values.append(abs(fx))
    return float(np.mean(values)) if values else 0.0


def average_fsr(samples):
    voltages = []
    forces = []
    for sample in samples:
        if not sample.get("hasFsr", False):
            continue
        voltages.append(float(sample.get("fsrVoltage", 0.0)))
        forces.append(float(sample.get("graspForceN", 0.0)))
    if not voltages:
        return None, None
    return float(np.mean(voltages)), float(np.mean(forces))


def ridge_fit(rows, outputs, alpha):
    x = np.asarray(rows, dtype=float)
    y = np.asarray(outputs, dtype=float)
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        raise ValueError("Calibration data contains NaN or infinite values.")
    x_mean = np.mean(x, axis=0)
    y_mean = float(np.mean(y))
    centered = x - x_mean
    weights = np.linalg.solve(
        centered.T @ centered + float(alpha) * np.eye(x.shape[1]),
        centered.T @ (y - y_mean),
    )
    bias = y_mean - float(x_mean @ weights)
    return weights, bias


def select_ridge_alpha(rows, outputs, groups):
    unique_groups = sorted(set(groups))
    if len(unique_groups) < 2:
        return DEFAULT_RIDGE_ALPHA

    x = np.asarray(rows, dtype=float)
    y = np.asarray(outputs, dtype=float)
    groups = np.asarray(groups)
    scores = []
    for alpha in RIDGE_ALPHAS:
        errors = []
        for group in unique_groups:
            train = groups != group
            weights, bias = ridge_fit(x[train], y[train], alpha)
            errors.extend((x[~train] @ weights + bias - y[~train]) ** 2)
        scores.append(float(np.mean(errors)))
    return float(RIDGE_ALPHAS[int(np.argmin(scores))])


def fit_emg_force(profile):
    required = ("emg_rest", *FORCE_KEYS, "bracing")
    repeats = sorted(
        {str(repeat) for key in required for repeat in profile.raw_repeats.get(key, {})}
    )
    missing = [
        (key, repeat)
        for repeat in repeats
        for key in required
        if not sample_windows(profile.raw_repeats.get(key, {}).get(repeat, []))
    ]
    if not repeats:
        missing = [(key, "") for key in required]
    if missing:
        labels = ", ".join(
            f"{CALIBRATION_LABELS[key]} (Repeat {repeat})"
            if repeat
            else CALIBRATION_LABELS[key]
            for key, repeat in missing
        )
        raise ValueError(f"Missing measurements: {labels}.")

    steady = {
        key: [
            sample
            for samples in profile.raw_repeats.get(key, {}).values()
            for sample in trim_samples(samples)
        ]
        for key in ("emg_rest", "left_100", "right_100", "bracing")
    }
    if steady["emg_rest"]:
        profile.emg_rest = mean_emg(steady["emg_rest"])
        (
            profile.emg_rest_fsr_voltage,
            profile.emg_rest_grasp_force_N,
        ) = average_fsr(steady["emg_rest"])
    if steady["left_100"]:
        profile.left_force_ref = peak_force(steady["left_100"], -1)
        profile.emg_left_mvc = robust_emg_peak(steady["left_100"])
    if steady["right_100"]:
        profile.right_force_ref = peak_force(steady["right_100"], 1)
        profile.emg_right_mvc = robust_emg_peak(steady["right_100"])
    if steady["bracing"]:
        profile.emg_bracing = robust_emg_peak(steady["bracing"])

    if not profile.emg_rest:
        raise ValueError("No valid EMG data in EMG Rest.")

    profile.emg_ref = compute_emg_ref(profile)
    if not profile.emg_ref:
        raise ValueError("No valid EMG data in MVC or Hold Stiff MVC.")

    rows, outputs, groups, trials = [], [], [], []
    valid_slots = None
    valid_conditions = set()

    for key in FORCE_KEYS:
        repeats = profile.raw_repeats.get(key, {})
        direction = -1.0 if key.startswith("left_") else 1.0

        for repeat, samples in repeats.items():
            for window_index, window_samples in enumerate(sample_windows(samples), 1):
                if valid_slots is None:
                    valid_slots = selected_emg_slots(window_samples[0])

                emg_rows = [
                    normalize_emg(
                        slot_emg_values(sample), profile.emg_rest, profile.emg_ref
                    )
                    for sample in window_samples
                ]
                emg_rows = [emg for emg in emg_rows if emg]
                force_mean = average_force(window_samples, direction)
                if not emg_rows or not valid_slots or force_mean <= 0.0:
                    continue

                emg_mean = np.mean(np.asarray(emg_rows, dtype=float), axis=0)
                u = [float(emg_mean[i]) for i in valid_slots]
                signed_force = direction * force_mean
                fsr_voltage, grasp_force = average_fsr(window_samples)
                rows.append(u)
                outputs.append(signed_force)
                groups.append(str(repeat))
                valid_conditions.add((key, str(repeat)))
                trials.append(
                    {
                        "key": key,
                        "repeat": str(repeat),
                        "window": window_index,
                        "force_mean": signed_force,
                        "emg_mean": u,
                        "fsr_voltage": fsr_voltage,
                        "grasp_force_N": grasp_force,
                    }
                )

    if not valid_slots:
        raise ValueError("No active EMG slots found in force measurements.")
    invalid = [
        (key, repeat)
        for repeat in repeats
        for key in FORCE_KEYS
        if (key, repeat) not in valid_conditions
    ]
    if invalid:
        labels = ", ".join(
            f"{CALIBRATION_LABELS[key]} (Repeat {repeat})" for key, repeat in invalid
        )
        raise ValueError(f"No valid EMG/directional force data: {labels}.")

    for repeat, samples in profile.raw_repeats.get("emg_rest", {}).items():
        for window_index, window_samples in enumerate(sample_windows(samples), 1):
            emg_rows = [
                normalize_emg(
                    slot_emg_values(sample), profile.emg_rest, profile.emg_ref
                )
                for sample in window_samples
            ]
            emg_rows = [emg for emg in emg_rows if emg]
            if not emg_rows:
                continue
            emg_mean = np.mean(np.asarray(emg_rows, dtype=float), axis=0)
            u = [float(emg_mean[i]) for i in valid_slots]
            fsr_voltage, grasp_force = average_fsr(window_samples)
            rows.append(u)
            outputs.append(0.0)
            groups.append(str(repeat))
            trials.append(
                {
                    "key": "rest",
                    "repeat": str(repeat),
                    "window": window_index,
                    "force_mean": 0.0,
                    "emg_mean": u,
                    "fsr_voltage": fsr_voltage,
                    "grasp_force_N": grasp_force,
                }
            )

    if not any(trial["key"] == "rest" for trial in trials):
        raise ValueError("No valid normalized EMG data in EMG Rest.")

    alpha = select_ridge_alpha(rows, outputs, groups)
    weights_valid, bias = ridge_fit(rows, outputs, alpha)

    weights = [float(weight) for weight in weights_valid]
    stiffness_scale = float(np.sum(np.abs(weights_valid)))

    rest_spi = [
        spi_co(trial["emg_mean"], weights) for trial in trials if trial["key"] == "rest"
    ]
    spi_ref = 1e-6

    bracing_u = normalize_emg(profile.emg_bracing, profile.emg_rest, profile.emg_ref)
    if bracing_u:
        trials.append(
            {
                "key": "bracing",
                "force_mean": 0.0,
                "emg_mean": [bracing_u[i] for i in valid_slots],
                "fsr_voltage": profile.bracing_fsr_voltage,
                "grasp_force_N": profile.bracing_grasp_force_N,
            }
        )

    bracing_rows = [
        normalize_emg(slot_emg_values(sample), profile.emg_rest, profile.emg_ref)
        for samples in profile.raw_repeats.get("bracing", {}).values()
        for sample in trim_samples(samples)
    ]
    bracing_spi = [
        spi_co([u[i] for i in valid_slots], weights) for u in bracing_rows if u
    ]
    if bracing_spi:
        spi_ref = float(
            np.percentile(np.asarray(bracing_spi, dtype=float), 95)
        )  # Use 95th percentile of bracing SPI as reference

    spi_rest = float(np.mean(rest_spi)) if rest_spi else 0.0
    profile.spi_rest = float(spi_rest)
    profile.spi_ref = float(max(spi_ref, spi_rest + 1e-6))
    profile.fit_discard_sec = FIT_DISCARD_SEC
    profile.fit_window_sec = FIT_WINDOW_SEC
    profile.ridge_alpha = alpha

    return weights, bias, stiffness_scale, trials, valid_slots


def force_norm(sample, profile):
    fx = force_value(sample)
    scale = profile.left_force_ref if fx < 0 else profile.right_force_ref
    if scale <= 0:
        scale = 50.0
    return clamp(fx / scale, -1.0, 1.0)
