import argparse
import json
import socket
import threading
import time
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

# ---------------------------- Load sub-modules --------------------------------
# -------------------------------------------------------------------------------
# calibration_flow: Manages the state and timing of the calibration process, including countdowns and recording periods.
# calibration_math: Contains functions for processing EMG and force data and fitting stiffness parameters.
# calibration_profile: Defines the CalibrationProfile data structure and functions for saving/loading profiles.
# -------------------------------------------------------------------------------
from calibration_flow import CalibrationFlow
from calibration_math import (
    average_force,
    compute_emg_ref,
    emg_values,
    fit_emg_force,
    force_norm,
    force_value,
    mean_emg,
    peak_force,
    position_norm,
    robust_emg_peak,
)
from calibration_profile import CalibrationProfile, save_profile

# -------------------------------------------------------------------------------


# ---------------------------- Setup TCP client --------------------------------
# Communication: TCP + UTF-8 + NDJSON (Newline-Delimited JSON)
class UnityTcpClient:
    def __init__(self, host, port, on_sample):
        self.host = host
        self.port = port
        self.on_sample = on_sample
        self.sock = None
        self.running = False
        self.connected = False

    def connect(self):
        if self.connected:
            return True
        try:
            self.sock = socket.create_connection((self.host, self.port), timeout=1.0)
            self.sock.settimeout(None)
            self.running = True
            self.connected = True
            threading.Thread(target=self.receive_loop, daemon=True).start()
            return True
        except OSError:
            self.close()
            return False

    def receive_loop(self):
        buffer = ""
        try:
            while self.running:
                data = self.sock.recv(
                    4096
                ).decode(
                    "utf-8"
                )  # Receive data from Unity, decode as UTF-8 string (Buffer size: 4096 bytes);
                if not data:
                    break
                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.lstrip("\ufeff")
                    if not line.strip():
                        continue
                    try:
                        self.on_sample(json.loads(line))
                    except json.JSONDecodeError:
                        continue
        finally:
            self.close()

    def send_response(self, profile):
        if not self.connected or self.sock is None:
            raise ConnectionError("Not connected to Unity.")
        self.sock.sendall((json.dumps(profile.response()) + "\n").encode("utf-8"))

    def close(self):
        self.running = False
        self.connected = False
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None


# -------------------------------------------------------------------------------


# ---------------------------- GUI and functions --------------------------------


class ParticipantBar(QWidget):
    def __init__(self):
        super().__init__()
        self.position_value = 0.0
        self.force_value = 0.0
        self.target_value = 0.0
        self.setMinimumHeight(180)

    def set_values(self, position_value, force_value, target_value):
        self.position_value = position_value
        self.force_value = force_value
        self.target_value = target_value
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        center = w / 2

        def draw_row(y, label, current, target):
            painter.setPen(QPen(QColor("#222222"), 1))
            painter.drawText(10, y - 10, label)
            painter.drawLine(20, y, w - 20, y)
            painter.setPen(QPen(QColor("#777777"), 1, Qt.DashLine))
            painter.drawLine(int(center), y - 14, int(center), y + 14)
            if abs(target) > 1e-6:
                tx = center + target * (w / 2 - 30)
                painter.setPen(QPen(QColor("#4caf50"), 10))
                painter.drawLine(int(tx - 12), y, int(tx + 12), y)
            cx = center + current * (w / 2 - 30)
            painter.setPen(QPen(QColor("#1e88e5"), 12))
            painter.drawPoint(int(cx), y)

        draw_row(60, "Handle Position", self.position_value, 0.0)
        draw_row(130, "Force Target", self.force_value, self.target_value)


class CalibrationApp(QWidget):
    STEP_INFO = {
        "emg_rest": ("Relax arm and hold the handle.", 0.0),
        "left_mvc": ("Push LEFT as hard as possible.", -1.0),
        "right_mvc": ("Push RIGHT as hard as possible.", 1.0),
        "bracing": (
            "Stay centered and maximally stiffen your arm without moving.",
            0.0,
        ),
    }

    def __init__(self, host, port):
        super().__init__()
        self.setWindowTitle("EMG Calibration App")
        self.profile = CalibrationProfile()
        self.flow = CalibrationFlow()
        self.tcp = UnityTcpClient(host, port, self.on_sample)
        self.last_sample = {}
        self.result_text = ""
        self.base_dir = Path(__file__).resolve().parent / "CalibrResults"
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.levels = (30, 50, 70)
        self.sample_count = 0
        self.last_connect_log = 0.0
        self.build_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(50)  # Update UI every 50 ms (20 FPS)

        self.reconnect_timer = QTimer()
        self.reconnect_timer.timeout.connect(self.ensure_connection)
        self.reconnect_timer.start(1000)
        self.ensure_connection()

    def build_ui(self):
        main_layout = QHBoxLayout(self)

        # -------------------------------------------------------------------------------
        # UI Layout:
        #   - left_layout: instructions, status, and real-time feedback
        #   - right_layout: controls for calibration steps, profile management, and logs
        # -------------------------------------------------------------------------------

        left_layout = QVBoxLayout()
        self.instruction_label = QLabel("Waiting for Unity connection.")
        self.instruction_label.setWordWrap(True)
        self.instruction_label.setFont(QFont("Arial", 18, QFont.Bold))
        self.countdown_label = QLabel("Ready")
        self.countdown_label.setAlignment(Qt.AlignCenter)
        self.countdown_label.setFont(QFont("Arial", 28, QFont.Bold))
        self.force_label = QLabel("Force: 0.000 N")
        self.position_label = QLabel("Position: 0.000 m")
        self.emg_label = QLabel("EMG: --")
        self.result_label = QLabel("Result: --")
        self.result_label.setWordWrap(True)
        self.debug_label = QLabel("TCP: disconnected | M2: false | samples: 0")
        self.display_bar = ParticipantBar()

        left_layout.addWidget(self.instruction_label)
        left_layout.addWidget(self.countdown_label)
        left_layout.addWidget(self.display_bar)
        left_layout.addWidget(self.force_label)
        left_layout.addWidget(self.position_label)
        left_layout.addWidget(self.emg_label)
        left_layout.addWidget(self.result_label)
        left_layout.addWidget(self.debug_label)

        right_layout = QVBoxLayout()
        right_layout.addWidget(QLabel("Participant ID"))
        self.participant_box = QLineEdit("P01")
        right_layout.addWidget(self.participant_box)

        base_buttons = [
            ("EMG Rest", lambda: self.start_step("emg_rest")),
            ("Left MVC", lambda: self.start_step("left_mvc")),
            ("Right MVC", lambda: self.start_step("right_mvc")),
            ("Bracing MVC", lambda: self.start_step("bracing")),
        ]
        for text, callback in base_buttons:
            btn = QPushButton(text)
            btn.clicked.connect(callback)
            right_layout.addWidget(btn)

        right_layout.addWidget(QLabel("Left Force Level"))
        self.left_level = QComboBox()
        self.left_level.addItems([f"{level}% MVC" for level in self.levels])
        # self.right_level.setCurrentIndex(0)  # Default to 30% for left side
        right_layout.addWidget(self.left_level)
        left_force_btn = QPushButton("Left Force")
        left_force_btn.clicked.connect(lambda: self.start_force_step(-1))
        right_layout.addWidget(left_force_btn)

        right_layout.addWidget(QLabel("Right Force Level"))
        self.right_level = QComboBox()
        self.right_level.addItems([f"{level}% MVC" for level in self.levels])
        # self.right_level.setCurrentIndex(0)  # Default to 30% for right side
        right_layout.addWidget(self.right_level)
        right_force_btn = QPushButton("Right Force")
        right_force_btn.clicked.connect(lambda: self.start_force_step(1))
        right_layout.addWidget(right_force_btn)

        fit_btn = QPushButton("Fit Stiffness")
        fit_btn.clicked.connect(self.fit_stiffness)
        right_layout.addWidget(fit_btn)
        load_btn = QPushButton("Load Data")
        load_btn.clicked.connect(self.load_profile)
        right_layout.addWidget(load_btn)
        send_btn = QPushButton("Send Parameters to Unity")
        send_btn.clicked.connect(self.send_params)
        right_layout.addWidget(send_btn)
        save_btn = QPushButton("Save Profile")
        save_btn.clicked.connect(self.save_profile)
        right_layout.addWidget(save_btn)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        right_layout.addWidget(self.log)

        main_layout.addLayout(left_layout, 3)
        main_layout.addLayout(right_layout, 2)

    # -------------------------------------------------------------------------------
    # Core functions:
    #   - ensure_connection: Manages TCP connection to Unity, with periodic retries and logging.
    #   - on_sample: Callback for incoming data samples from Unity, updates state and UI.
    #   - start_step/start_force_step: Initiates calibration steps based on user actions.
    #   - finish_step: Processes recorded samples to update the calibration profile and results.
    #   - fit_stiffness: Computes stiffness parameters based on collected data and updates the profile.
    #   - send_params: Sends the calibrated parameters back to Unity.
    #   - load_profile/save_profile: Handles loading and saving of calibration profiles to/from JSON files.
    #
    # Auxiliary functions:
    #   - update_ui: Refreshes the UI elements based on the latest data and state.
    #   - append_log: Utility to add timestamped entries to the log display.
    #   - closeEvent: Ensures TCP connection is closed when the application exits.
    # -------------------------------------------------------------------------------

    def ensure_connection(self):
        if self.tcp.connected:
            return
        if self.tcp.connect():
            self.append_log("Connected to Unity CalibrationManager.")
            self.instruction_label.setText("Connected. Ready for calibration.")
        elif time.time() - self.last_connect_log > 3.0:
            self.append_log("Waiting for Unity CalibrationManager...")
            self.last_connect_log = time.time()

    def on_sample(self, sample):
        self.last_sample = sample
        self.sample_count += 1
        self.flow.on_sample(sample)

    def start_step(self, key):
        instruction, target = self.STEP_INFO[key]
        ok, message = self.flow.start(key, instruction, target)
        if ok:
            self.instruction_label.setText(instruction)
        self.append_log(message)

    def start_force_step(self, direction):
        scale = (
            self.profile.left_force_ref
            if direction < 0
            else self.profile.right_force_ref
        )
        if scale <= 0:
            self.append_log("Run MVC first.")
            return

        combo = self.left_level if direction < 0 else self.right_level
        level = self.levels[combo.currentIndex()]
        side = "LEFT" if direction < 0 else "RIGHT"
        key = f"{side.lower()}_{level}"
        target = (level / 100.0) * (-1.0 if direction < 0 else 1.0)
        instruction = f"Maintain approximately {level}% of {side} MVC."
        ok, message = self.flow.start(key, instruction, target)
        if ok:
            self.instruction_label.setText(instruction)
        self.append_log(message)

    def finish_step(self, key, samples):
        raw_key = {"left_mvc": "left_100", "right_mvc": "right_100"}.get(key, key)
        self.profile.raw[raw_key] = samples
        if key == "emg_rest":
            self.profile.emg_rest = mean_emg(samples)
            self.profile.note = "EMG rest calibrated."
            self.result_text = "EMG rest: " + ", ".join(
                f"Ch{i + 1}={value:.6g}"
                for i, value in enumerate(self.profile.emg_rest)
            )
        elif key == "left_mvc":
            self.profile.left_force_ref = peak_force(samples, -1)
            self.profile.emg_left_mvc = robust_emg_peak(samples)
            self.profile.note = "Left MVC calibrated."
            self.result_text = f"Left MVC = {self.profile.left_force_ref:.3f} N"
        elif key == "right_mvc":
            self.profile.right_force_ref = peak_force(samples, 1)
            self.profile.emg_right_mvc = robust_emg_peak(samples)
            self.profile.note = "Right MVC calibrated."
            self.result_text = f"Right MVC = {self.profile.right_force_ref:.3f} N"
        elif key == "bracing":
            self.profile.emg_bracing = robust_emg_peak(samples)
            self.profile.note = "Bracing MVC calibrated."
            self.result_text = "Bracing: " + ", ".join(
                f"Ch{i + 1}={value:.6g}"
                for i, value in enumerate(self.profile.emg_bracing)
            )
        elif key.startswith("left_"):
            level = key.split("_")[1]
            mean_force = average_force(samples, -1)
            self.profile.note = f"Left {level}% force recorded."
            self.result_text = f"Left {level}% mean force = {mean_force:.3f} N"
        elif key.startswith("right_"):
            level = key.split("_")[1]
            mean_force = average_force(samples, 1)
            self.profile.note = f"Right {level}% force recorded."
            self.result_text = f"Right {level}% mean force = {mean_force:.3f} N"

        if (
            self.profile.emg_left_mvc
            and self.profile.emg_right_mvc
            and self.profile.emg_bracing
        ):
            self.profile.emg_ref = compute_emg_ref(self.profile)

        self.result_label.setText(f"Result: {self.result_text}")
        self.append_log(self.result_text)

    def fit_stiffness(self):
        weights, force_bias, stiffness_scale, standby_k, fit_trials, fit_slots = (
            fit_emg_force(self.profile)
        )
        if not weights:
            self.append_log("Need MVC and force-level datasets before fitting.")
            return
        self.profile.emg_force_weights = weights
        self.profile.emg_force_bias = force_bias
        self.profile.fit_trials = fit_trials
        self.profile.fit_slots = fit_slots
        # stiffness_scale = sum(abs(w_i)); regression sensitivity, not physical stiffness.
        self.profile.stiffness_scale = stiffness_scale
        self.profile.standbyK = standby_k
        self.profile.note = "Stiffness fitted."
        self.result_text = f"stiffness_scale = {stiffness_scale:.3f}"
        self.result_label.setText(f"Result: {self.result_text}")
        self.append_log(self.result_text)

    def send_params(self):
        try:
            self.tcp.send_response(self.profile)
            self.append_log("Parameters sent to Unity.")
        except Exception as ex:
            self.append_log(f"Failed to send parameters: {ex}")

    def load_profile(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Load Calibration JSON",
            str(self.base_dir),
            "JSON files (*.json);;All files (*)",
        )
        if not path:
            return

        try:
            with open(path, "r", encoding="utf-8-sig") as f:
                data = json.load(f)

            profile = CalibrationProfile()
            for key, value in data.items():
                if hasattr(profile, key):
                    setattr(profile, key, value)

            self.profile = profile
            self.participant_box.setText(profile.participant_id or Path(path).stem)
            self.result_text = (
                f"Loaded {Path(path).name}; "
                f"weights={len(profile.emg_force_weights)}, bias={profile.emg_force_bias:.3f}"
            )
            self.result_label.setText(f"Result: {self.result_text}")
            self.append_log(self.result_text)
        except Exception as ex:
            self.append_log(f"Failed to load profile: {ex}")

    def save_profile(self):
        participant_id, path = save_profile(
            self.profile, str(self.base_dir), self.participant_box.text()
        )
        self.participant_box.setText(participant_id)
        self.append_log(f"Saved profile: {path}")

    def update_ui(self):
        finished = self.flow.update()
        if finished:
            self.finish_step(*finished)
            self.instruction_label.setText("Ready.")

        sample = self.last_sample
        fx = force_value(sample)
        pos = sample.get("m2Position", [])
        pos_x = float(pos[0]) if len(pos) > 0 else 0.0
        emg = emg_values(sample)
        has_m2 = bool(sample.get("hasM2", False))
        target = self.flow.target
        direction = -1 if target < 0 else 1
        force_bar = (
            force_norm(sample, self.profile, direction) if abs(target) > 1e-6 else 0.0
        )

        self.force_label.setText(f"Force: {fx:.3f} N")
        self.position_label.setText(f"Position: {pos_x:.3f} m")
        self.emg_label.setText(
            "EMG: "
            + (
                ", ".join(f"{v:.6f}" if abs(v) >= 1e-4 else f"{v:.3e}" for v in emg[:8])
                if emg
                else "--"
            )
        )
        self.result_label.setText(f"Result: {self.result_text or '--'}")
        self.debug_label.setText(
            f"TCP: {'connected' if self.tcp.connected else 'disconnected'} | "
            f"M2: {str(has_m2).lower()} | samples: {self.sample_count} | latest force: {fx:.3f}"
        )
        self.display_bar.set_values(position_norm(sample), force_bar, target)

        if self.flow.state == "countdown":
            self.countdown_label.setText(f"Countdown: {self.flow.remaining:.1f}s")
        elif self.flow.state == "recording":
            self.countdown_label.setText(f"Recording: {self.flow.remaining:.1f}s")
        else:
            self.countdown_label.setText("Ready")

    def append_log(self, text):
        stamp = time.strftime("%H:%M:%S")
        self.log.append(f"[{stamp}] {text}")

    def closeEvent(self, event):
        self.tcp.close()
        super().closeEvent(event)


# -------------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=25001)
    args = parser.parse_args()

    app = QApplication([])
    window = CalibrationApp(args.host, args.port)
    window.resize(1200, 720)
    window.show()
    app.exec()  # app.exec(): Starts the Qt event loop, which keeps the application running and responsive until it is closed.


# this is a common Python idiom: It prevents code from running unexpectedly when you import a file into another script.
if __name__ == "__main__":
    main()
