"""Independent RVR encoder distance conversion; no Locator/IMU dependency."""
import json
import math
from pathlib import Path
import time

FIELDS = ("LeftTicks", "RightTicks")
DEFAULT_CALIBRATION = Path("/etc/a2r-robot/encoder_calibration.json")


def tick_delta(value, origin):
    """Signed delta of uint32 counters; samples must be <2**31 ticks apart."""
    return (int(value) - int(origin) + 2**31) % 2**32 - 2**31


class EncoderDistance:
    def __init__(self, path=DEFAULT_CALIBRATION):
        self.path = Path(path)
        self.scales = None
        self.method = "unavailable"
        self.notice = "Not calibrated: measure a straight forward floor run."
        self.previous = None
        self.timestamp = None
        self.last_motion = None
        self.ticks = [0, 0]
        self.absolute_ticks = [0, 0]
        self.delta = [0, 0]
        self.dt = None
        self.calibration_start = None
        try:
            saved = json.loads(self.path.read_text())
            scales = [float(saved["signed_metres_per_tick"][side]) for side in ("left", "right")]
            if saved["version"] != 1 or not all(math.isfinite(s) and 0 < abs(s) < 1 for s in scales):
                raise ValueError("invalid calibration scale/version")
            self.scales = scales
            self.method = saved.get("method", "externally measured straight forward ground run")
            self.notice = ("PROVISIONAL: 4337.768 ticks/m from another RVR; validate on this robot."
                           if self.method == "published reference, unvalidated on this robot" else
                           "Loaded measured encoder calibration. Positive distance = forward.")
        except FileNotFoundError:
            pass
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.notice = f"Calibration unavailable: {exc}"

    def update(self, encoders, timestamp):
        values = [int(encoders[f]) for f in FIELDS]
        self.dt = None if self.timestamp is None else timestamp - self.timestamp
        self.delta = ([0, 0] if self.previous is None else
                      [tick_delta(v, p) for v, p in zip(values, self.previous)])
        for i, delta in enumerate(self.delta):
            self.ticks[i] += delta
            self.absolute_ticks[i] += abs(delta)
        if any(self.delta):
            self.last_motion = timestamp
        self.previous, self.timestamp = values, timestamp

    def begin_calibration(self, now):
        if self.timestamp is None or now - self.timestamp > 0.5:
            raise ValueError("Need fresh encoder data before starting.")
        if self.last_motion is not None and now - self.last_motion < 0.4:
            raise ValueError("Stop both treads before starting the measured run.")
        self.calibration_start = (self.ticks.copy(), self.absolute_ticks.copy())
        self.notice = "Recording: drive straight FORWARD along the measured path, stop, then save."

    def finish_calibration(self, distance_m, now):
        if self.calibration_start is None:
            raise ValueError("Press Start measured run first.")
        if not math.isfinite(distance_m) or distance_m < 0.1:
            raise ValueError("Measured forward distance must be at least 0.1 m.")
        if self.timestamp is None or now - self.timestamp > 0.5:
            raise ValueError("Encoder data is stale; calibration not saved.")
        if self.last_motion is not None and now - self.last_motion < 0.4:
            raise ValueError("Stop both treads and wait a moment before saving.")
        starts, absolute_starts = self.calibration_start
        counts = [a-b for a, b in zip(self.ticks, starts)]
        travel = [a-b for a, b in zip(self.absolute_ticks, absolute_starts)]
        if any(abs(c) < 10 for c in counts):
            raise ValueError("Too few ticks on one or both treads; run not saved.")
        if any(t > abs(c) * 1.1 for t, c in zip(travel, counts)):
            raise ValueError("Tread reversal detected. Restart and drive only forward.")
        scales = [distance_m / c for c in counts]
        if not all(0 < abs(s) < 1 for s in scales):
            raise ValueError("Implausible scale; check the measured distance and ticks.")
        saved = {"version": 1, "created_unix_time": time.time(),
                 "method": "externally measured straight forward ground run",
                 "measured_distance_m": distance_m,
                 "run_ticks": dict(zip(("left", "right"), counts)),
                 "signed_metres_per_tick": dict(zip(("left", "right"), scales))}
        temporary = self.path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(saved, indent=2) + "\n")
        temporary.replace(self.path)
        self.scales = scales
        self.method = saved["method"]
        self.calibration_start = None
        self.notice = "Calibration saved for this robot. Distances are since this connection."
        return saved

    def reading(self):
        if self.scales is None:
            return None
        distance = [c*s for c, s in zip(self.ticks, self.scales)]
        increments = [c*s for c, s in zip(self.delta, self.scales)]
        # Do not report an average speed across a long input outage as live speed.
        valid_speed = self.dt is not None and 0 < self.dt <= 0.5
        speed = [d/self.dt for d in increments] if valid_speed else [None, None]
        return {"left_distance_m": distance[0], "right_distance_m": distance[1],
                "center_distance_m": sum(distance)/2,
                "left_delta_m": increments[0], "right_delta_m": increments[1],
                "left_speed_m_s": speed[0], "right_speed_m_s": speed[1],
                "sample_dt_s": self.dt,
                "scale_method": self.method,
                "signed_metres_per_tick": dict(zip(("left", "right"), self.scales))}
