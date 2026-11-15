"""Vita takeover arbitration for autonomous_land: one DroneLink, no competing writers."""
import threading
import time

from bridge import VitaBridge
from drone import SPEED_PRESETS

NEUTRAL = dict(roll=0., pitch=0., throttle=0., yaw=0.)
TAKEOVER_THRESHOLD = 0.18
NEUTRAL_AFTER = 0.3
LAND_AFTER = 1.5


class _VitaInput:
    """Adapter for the existing bridge; flight actions go through the authority lock."""
    def __init__(self, owner):
        self.owner = owner

    def set_sticks(self, **sticks):
        self.owner.receive_sticks(sticks)

    @property
    def max_offset(self):
        return self.owner._manual_offsets

    @max_offset.setter
    def max_offset(self, value):
        with self.owner._lock:
            self.owner._manual_offsets = value
            if self.owner._mode == "MANUAL":
                self.owner.raw.max_offset = value

    def land(self):
        return self.owner.request_land()

    def emergency_stop(self):
        self.owner.emergency_stop()

    def takeoff(self):
        return self.owner.takeoff()

    def calibrate(self):
        return False  # never calibrate during an autonomous/takeover flight

    def adjust_trim(self, *args):
        pass  # keep the same saved trim across the handoff

    def set_trim(self, **kwargs):
        pass

    def get_trim(self):
        return self.owner.raw.get_trim()

    def status(self):
        status = self.owner.raw.status()
        status["heartbeat_age"] = self.owner.vita_age()
        return status


class _TakeoverBridge(VitaBridge):
    def handle_packet(self, data, addr, now=None):
        # Keep the first validated peer for this session; do not switch pilots on an unrelated packet.
        if self.vita_ip is not None and addr[0] != self.vita_ip:
            return False
        return super().handle_packet(data, addr, now)

    def send_telemetry(self, now=None):
        # run() calls this after every packet or socket timeout, independent of camera processing.
        self.link.owner.tick()
        super().send_telemetry(now)


class VitaTakeover:
    """Manual takeoff, explicit engagement, stick takeover; LAND/EMERGENCY are terminal."""
    def __init__(self, raw, *, port=9000, bind="0.0.0.0", preview=False, log=print):
        self.raw, self.preview, self._log = raw, preview, log
        self._lock = threading.RLock()
        self._mode = "AUTO"
        self._last_rx = None
        self._manual = NEUTRAL.copy()
        self._manual_offsets = SPEED_PRESETS[1]
        self._started = False
        self.takeoff_enabled = False
        self._landing_accepted = False
        self._thread = None
        self.bridge = _TakeoverBridge(_VitaInput(self), port=port, bind=bind, log=log)

    def __getattr__(self, name):
        return getattr(self.raw, name)

    @property
    def mode(self):
        with self._lock:
            return self._mode

    @property
    def manual_command(self):
        with self._lock:
            return self._manual.copy() if self.vita_age() <= NEUTRAL_AFTER else NEUTRAL.copy()

    def vita_age(self):
        with self._lock:
            return 999. if self._last_rx is None else max(0., time.monotonic() - self._last_rx)

    def start(self):
        self.raw.start()
        self._thread = threading.Thread(target=self.bridge.run, name="vita-takeover", daemon=True)
        self._thread.start()
        self._log(f"Vita takeover on UDP {self.bridge.port}: move a stick to latch MANUAL; "
                  "Cross=land, Circle=motor cut. Triangle/PC T=manual takeoff; PC A=engage landing.")

    def receive_sticks(self, sticks):
        with self._lock:
            self._last_rx = time.monotonic()
            self._manual = dict(sticks)
            if (self._mode == "AUTO" and (self._started or self.preview)
                    and max(abs(v) for v in sticks.values()) >= TAKEOVER_THRESHOLD):
                self._mode = "MANUAL"
                self.raw.max_offset = self._manual_offsets
                self._log("\nVITA MANUAL TAKEOVER: autonomy disabled for this run.")
            if self._mode == "MANUAL":
                self.raw.set_sticks(**sticks)

    def takeoff(self):
        with self._lock:
            if (not self.takeoff_enabled or self._started or self._mode != "AUTO" or self.vita_age() > NEUTRAL_AFTER
                    or max(abs(v) for v in self._manual.values()) >= TAKEOVER_THRESHOLD):
                self._log("Takeoff refused: Vita must be connected, centred and not in a stop/manual state.")
                return False
            accepted = self.raw.takeoff()
            if accepted:
                self._started = True
                self._mode = "MANUAL"
                self.raw.max_offset = self._manual_offsets
            return accepted

    @property
    def started(self):
        with self._lock:
            return self._started

    def engage(self):
        """Caller checks the camera; authority and centred-controller checks are atomic here."""
        with self._lock:
            if (not self._started or self._mode != "MANUAL" or self.vita_age() > NEUTRAL_AFTER
                    or max(abs(v) for v in self._manual.values()) >= TAKEOVER_THRESHOLD):
                return False
            self.raw.set_sticks(**NEUTRAL)
            self.raw.max_offset = SPEED_PRESETS[1]
            self._mode = "AUTO"
            return True

    def set_sticks(self, **sticks):
        with self._lock:
            self._tick_locked()
            if self._mode == "AUTO":
                self.raw.set_sticks(**(sticks if not self._started or self.vita_age() <= NEUTRAL_AFTER else NEUTRAL))

    def heartbeat(self):
        with self._lock:
            self._tick_locked()
            if self._mode == "AUTO" and (not self._started or self.vita_age() <= NEUTRAL_AFTER):
                self.raw.heartbeat()

    def land(self):
        """Autonomous request: never override a pilot who has taken control."""
        with self._lock:
            if self._mode in ("MANUAL", "EMERGENCY"):
                return False
            return self.request_land()

    def request_land(self):
        """Operator request, also used by the independent Vita-link watchdog."""
        with self._lock:
            if self._mode == "EMERGENCY":
                return False
            if self._mode != "LAND":
                self._mode = "LAND"
                self._log("\nLanding requested; stick takeover cannot cancel firmware landing.")
            self.raw.set_sticks(**NEUTRAL)
            if not self._landing_accepted and self._started:
                self._landing_accepted = bool(self.raw.land())
            return self._landing_accepted

    def emergency_stop(self):
        with self._lock:
            self._mode = "EMERGENCY"
            self.raw.emergency_stop()

    def _tick_locked(self):
        age = self.vita_age()
        if self._started and self._mode in ("AUTO", "MANUAL"):
            if age > LAND_AFTER:
                self.request_land()
            elif age > NEUTRAL_AFTER:
                self.raw.set_sticks(**NEUTRAL)
        if self._mode == "LAND":
            self.request_land()  # retry a request rejected while a takeoff pulse was still pending

    def tick(self):
        with self._lock:
            self._tick_locked()

    def close(self, stop_motors=False):
        self.bridge.close()
        if self._thread is not None:
            self._thread.join(timeout=1)
        self.raw.close(stop_motors=stop_motors)

