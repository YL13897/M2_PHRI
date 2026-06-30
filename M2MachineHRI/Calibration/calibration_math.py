import numpy as np

# -------------------------------------------------------------------------------
# calibration_math.py contains functions for processing raw calibration data,
# including normalization, fitting, and computing reference values.
# -------------------------------------------------------------------------------


CENTER_X = 0.32
HALF_RANGE_X = 0.18


def clamp(value, low, high):
    return max(low, min(high, value))


def position_norm(sample):
    pos = sample.get("m2Position", []) if sample else []
    if len(pos) < 1:
        return 0.0
    return clamp((float(pos[0]) - CENTER_X) / HALF_RANGE_X, -1.0, 1.0)


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
    for channel in slots:
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


def mean_emg(samples):
    rows = [slot_emg_values(sample) for sample in samples if slot_emg_values(sample)]
    if not rows:
        return []
    return np.mean(np.asarray(rows, dtype=float), axis=0).tolist()


# Robust peak estimation using the 95th percentile to mitigate outliers.
def robust_emg_peak(samples, q=95):
    rows = [slot_emg_values(sample) for sample in samples if slot_emg_values(sample)]
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


def fit_emg_force(profile):
    if not profile.emg_rest:
        return [], 0.0, 0.0, 0.0, [], []

    profile.emg_ref = compute_emg_ref(profile)
    if not profile.emg_ref:
        return [], 0.0, 0.0, 0.0, [], []

    rows = []
    outputs = []
    trials = []
    valid_slots = None

    for key, samples in profile.raw.items():
        if key.startswith("left_"):
            direction = -1.0
        elif key.startswith("right_"):
            direction = 1.0
        else:
            continue

        if not samples:
            continue
        slots = samples[0].get("emgSlots", []) if samples[0] else []
        if valid_slots is None:
            valid_slots = [i for i, ch in enumerate(slots) if ch > 0]

        emg_rows = [
            normalize_emg(slot_emg_values(sample), profile.emg_rest, profile.emg_ref)
            for sample in samples
        ]
        emg_rows = [emg for emg in emg_rows if emg]
        if not emg_rows or not valid_slots:
            continue

        emg_mean = np.mean(np.asarray(emg_rows, dtype=float), axis=0).tolist()
        force_mean = average_force(samples, direction)
        if force_mean <= 0.0:
            continue

        row = [emg_mean[i] for i in valid_slots]
        row.append(1.0)
        rows.append(row)
        signed_force = direction * force_mean
        outputs.append(signed_force)
        trials.append(
            {
                "key": key,
                "force_mean": signed_force,
                "emg_mean": [emg_mean[i] for i in valid_slots],
            }
        )

    if not rows or not valid_slots:
        return [], 0.0, 0.0, 0.0, [], []

    for key, emg in (("rest", profile.emg_rest), ("bracing", profile.emg_bracing)):
        u = normalize_emg(emg, profile.emg_rest, profile.emg_ref)
        if not u:
            continue
        row = [u[i] for i in valid_slots]
        row.append(1.0)
        rows.append(row)
        outputs.append(0.0)
        trials.append(
            {
                "key": key,
                "force_mean": 0.0,
                "emg_mean": [u[i] for i in valid_slots],
            }
        )

    # Perform linear regression to fit EMG to force, with a bias term.
    x = np.asarray(rows, dtype=float)
    y = np.asarray(outputs, dtype=float)
    coeffs, *_ = np.linalg.lstsq(x, y, rcond=None)
    weights_valid = coeffs[:-1]
    bias = float(coeffs[-1])

    weights = [float(weight) for weight in weights_valid]
    stiffness_scale = float(np.sum(np.abs(weights_valid)))
    standby_k = max(
        10.0, 1200.0 * stiffness_scale
    )  # Minimum standbyK of 10.0 to ensure some responsiveness even with low EMG-force sensitivity.

    spi_rest = 0.0
    spi_ref = 1e-6
    for trial in trials:
        emg_term = float(
            np.dot(
                np.asarray(weights, dtype=float),
                np.asarray(trial["emg_mean"], dtype=float),
            )
        )
        spi = float(
            np.dot(
                np.abs(np.asarray(weights, dtype=float)),
                np.asarray(trial["emg_mean"], dtype=float),
            )
            + bias
        )
        spi_new = float(spi_co(trial["emg_mean"], weights))
        trial["emg_term"] = emg_term
        trial["force_pred"] = emg_term + bias
        trial["spi_pred"] = spi
        if trial["key"] == "rest":
            spi_rest = spi_new

    bracing_rows = [
        normalize_emg(slot_emg_values(sample), profile.emg_rest, profile.emg_ref)
        for sample in profile.raw.get("bracing", [])
    ]
    bracing_spi = [spi_co([u[i] for i in valid_slots], weights) for u in bracing_rows if u]
    if bracing_spi:
        spi_ref = float(np.percentile(np.asarray(bracing_spi, dtype=float), 80))

    profile.spi_rest = float(spi_rest)
    profile.spi_ref = float(max(spi_ref, spi_rest + 1e-6))

    return weights, bias, stiffness_scale, standby_k, trials, valid_slots

def force_norm(sample, profile, direction):
    scale = profile.left_force_ref if direction < 0 else profile.right_force_ref
    fx = force_value(sample)
    if scale > 0:
        value = abs(fx) / scale
    else:  # Default to 50N if no reference (used in the 100% MVC calibration)
        value = abs(fx) / 50.0
    return clamp(value, 0.0, 1.2) * (1.0 if direction >= 0 else -1.0)
