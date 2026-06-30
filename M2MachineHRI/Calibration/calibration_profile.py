import json
import os
from dataclasses import dataclass, field

# -------------------------------------------------------------------------------
# calibration_profile.py defines the CalibrationProfile dataclass to store calibration data,
# and functions to save profiles to disk with unique participant IDs.
# -------------------------------------------------------------------------------


@dataclass
class CalibrationProfile:
    participant_id: str = "P01"
    emg_rest: list = field(default_factory=list)
    emg_ref: list = field(default_factory=list)
    left_force_ref: float = 0.0
    right_force_ref: float = 0.0
    emg_left_mvc: list = field(default_factory=list)
    emg_right_mvc: list = field(default_factory=list)
    emg_bracing: list = field(default_factory=list)
    emg_force_weights: list = field(default_factory=list)
    emg_force_bias: float = 0.0
    spi_rest: float = 0.0
    spi_ref: float = 1.0
    fit_slots: list = field(default_factory=list)
    fit_trials: list = field(default_factory=list)
    stiffness_scale: float = 0.0
    standbyK: float = 0.0
    note: str = ""
    raw: dict = field(default_factory=dict)

    def response(self):
        calib_force = max(self.left_force_ref, self.right_force_ref)
        return {
            "calibForce": calib_force,
            "standbyK": self.standbyK,
            "emgScale": self.stiffness_scale,
            "threshold": 0.0,
            "emgRest": self.emg_rest,
            "emgBracing": self.emg_bracing,
            "emgRef": self.emg_ref,
            "emgForceWeights": self.emg_force_weights,
            "emgForceBias": self.emg_force_bias,
            "spiRest": self.spi_rest,
            "spiRef": self.spi_ref,
            "note": self.note,
        }


def next_profile_path(directory, base_id):
    base_id = (base_id or "P01").strip() or "P01"
    prefix = base_id[:1] if base_id[:1].isalpha() else "P"
    digits = base_id[1:] if base_id[:1].isalpha() else base_id
    index = int(digits) if digits.isdigit() else 1

    while True:
        participant_id = f"{prefix}{index:02d}"
        filename = f"{participant_id}_calibration.json"
        path = os.path.join(directory, filename)
        if not os.path.exists(path):
            return participant_id, path
        index += 1


def save_profile(profile, directory, base_id):
    participant_id, path = next_profile_path(directory, base_id)
    profile.participant_id = participant_id
    data = {
        "participant_id": profile.participant_id,
        "emg_rest": profile.emg_rest,
        "emg_ref": profile.emg_ref,
        "left_force_ref": profile.left_force_ref,
        "right_force_ref": profile.right_force_ref,
        "emg_left_mvc": profile.emg_left_mvc,
        "emg_right_mvc": profile.emg_right_mvc,
        "emg_bracing": profile.emg_bracing,
        "emg_force_weights": profile.emg_force_weights,
        "emg_force_bias": profile.emg_force_bias,
        "spi_rest": profile.spi_rest,
        "spi_ref": profile.spi_ref,
        "fit_slots": profile.fit_slots,
        "fit_trials": profile.fit_trials,
        "stiffness_scale": profile.stiffness_scale,
        "standbyK": profile.standbyK,
        "note": profile.note,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    return participant_id, path

