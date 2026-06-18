import time

# -------------------------------------------------------------------------------
# CalibrationFlow manages the state and timing of the calibration process,
# including countdowns and recording periods.
# -------------------------------------------------------------------------------


class CalibrationFlow:
    def __init__(self, countdown_sec=3.0, record_sec=3.0):
        self.countdown_sec = countdown_sec
        self.record_sec = record_sec
        self.state = "idle"
        self.key = ""
        self.instruction = "Connect to Unity to begin."
        self.target = 0.0
        self.buffer = []
        self.latest_sample = {}
        self.until = 0.0

    @property
    def is_running(self):
        return self.state != "idle"

    @property
    def remaining(self):
        return max(0.0, self.until - time.time()) if self.is_running else 0.0

    def start(self, key, instruction, target=0.0):
        if self.is_running:
            return False, "Calibration already running."
        self.state = "countdown"
        self.key = key
        self.instruction = instruction
        self.target = target
        self.buffer = []
        self.until = time.time() + self.countdown_sec
        return True, f"{instruction} Countdown started."

    def on_sample(self, sample):
        self.latest_sample = sample
        if self.state == "recording":
            self.buffer.append(sample)

    def update(self):
        if not self.is_running or time.time() < self.until:
            return None
        if self.state == "countdown":
            self.state = "recording"
            self.until = time.time() + self.record_sec
            return None
        result = (self.key, list(self.buffer))
        self.reset()
        return result

    def reset(self):
        self.state = "idle"
        self.key = ""
        self.instruction = "Ready."
        self.target = 0.0
        self.buffer = []
        self.until = 0.0
