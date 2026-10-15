
"""bridge.py - Vita controller bridge for the WIFI_CAM drone.

Optional ORB-SLAM3 integration:
  --slam                  Enable live monocular SLAM.
  --slam-pose-port 5010   Local UDP port receiving JSON pose messages.

Safety: every valid Vita packet is a heartbeat for DroneLink's failsafe.
If the Vita stops sending, the drone goes neutral, lands, and eventually
cuts the motors according to the configured failsafe timings.
"""

import argparse
import os
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path

import vitalink as vl
from drone import SPEED_PRESETS

SPEED_OFFSETS = SPEED_PRESETS
B = vl.BUTTONS

TRIM_BUTTONS = {
    B["TRIM_PITCH_UP"]: ("pitch", +1),
    B["TRIM_PITCH_DOWN"]: ("pitch", -1),
    B["TRIM_ROLL_LEFT"]: ("roll", -1),
    B["TRIM_ROLL_RIGHT"]: ("roll", +1),
    B["TRIM_YAW_LEFT"]: ("yaw", -1),
    B["TRIM_YAW_RIGHT"]: ("yaw", +1),
}


class VitaBridge:
    def __init__(
        self,
        link,
        port=vl.DEFAULT_PORT,
        bind="0.0.0.0",
        telemetry_port=vl.VITA_PORT,
        telemetry_hz=5.0,
        trim_step=1,
        on_trim_change=None,
        log=print,
    ):
        self.link = link
        self.telemetry_port = telemetry_port
        self.telemetry_period = 1.0 / telemetry_hz
        self.trim_step = trim_step
        self.on_trim_change = on_trim_change
        self._log = log

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind, port))
        self.sock.settimeout(0.05)
        self.port = self.sock.getsockname()[1]

        self._running = False
        self._vita_ip = None
        self._last_rx = None
        self._last_seq = None
        self._prev_buttons = None
        self._last_emergency = 0.0
        self._next_telemetry = 0.0
        self._fps_time = time.monotonic()
        self._fps_frames = 0
        self._fps = 0
        self.packets_ok = self.packets_bad = self.packets_stale = 0

    @property
    def vita_ip(self):
        """Address of the Vita after it sends a valid packet."""
        return self._vita_ip

    # ------------------------------------------------------------ incoming
    def handle_packet(self, data, addr, now=None):
        """Process one control packet. Returns True if it was applied."""
        now = time.monotonic() if now is None else now
        msg = vl.parse_control(data)

        if msg is None:
            self.packets_bad += 1
            return False

        if self._last_rx is None or now - self._last_rx > 1.0:
            # New session.
            self._last_seq = None
            self._prev_buttons = None
            self._log(f"[bridge] Vita connected from {addr[0]}")

        if self._last_seq is not None:
            diff = (msg["seq"] - self._last_seq) & 0xFF
            if diff == 0 or diff > 128:
                self.packets_stale += 1
                return False

        self._last_seq = msg["seq"]
        self._last_rx = now
        self._vita_ip = addr[0]
        self.packets_ok += 1

        self.link.set_sticks(
            roll=msg["roll"] / 127.0,
            pitch=msg["pitch"] / 127.0,
            throttle=msg["throttle"] / 127.0,
            yaw=msg["yaw"] / 127.0,
        )

        if msg["speed"] in SPEED_OFFSETS:
            self.link.max_offset = SPEED_OFFSETS[msg["speed"]]

        buttons = msg["buttons"]
        prev = self._prev_buttons

        if prev is None:
            # Ignore buttons already held at connection, except EMERGENCY.
            prev = buttons & ~B["EMERGENCY"]

        edges = buttons & ~prev
        self._prev_buttons = buttons

        if buttons & B["EMERGENCY"] and (
            edges & B["EMERGENCY"] or now - self._last_emergency > 1.0
        ):
            self._last_emergency = now
            self.link.emergency_stop()
            self._log("[bridge] EMERGENCY STOP")

        if edges & B["TAKEOFF"]:
            self.link.takeoff()
            self._log("[bridge] takeoff")

        if edges & B["LAND"]:
            self.link.land()
            self._log("[bridge] land")

        if edges & B["CALIBRATE"]:
            self.link.calibrate()
            self._log("[bridge] calibrate")

        changed = False
        for bit, (axis, direction) in TRIM_BUTTONS.items():
            if edges & bit:
                self.link.adjust_trim(axis, direction * self.trim_step)
                changed = True

        if edges & B["TRIM_RESET"]:
            self.link.set_trim(roll=0, pitch=0, yaw=0)
            changed = True

        if changed and self.on_trim_change:
            self.on_trim_change(self.link.get_trim())

        return True

    # ------------------------------------------------------------ outgoing
    def send_telemetry(self, now=None):
        now = time.monotonic() if now is None else now

        if self._vita_ip is None or now < self._next_telemetry:
            return

        self._next_telemetry = now + self.telemetry_period
        st = self.link.status()

        if now - self._fps_time >= 1.0:
            elapsed = now - self._fps_time
            self._fps = int(
                round((st["frames_ok"] - self._fps_frames) / elapsed)
            )
            self._fps_time = now
            self._fps_frames = st["frames_ok"]

        msg = vl.pack_telemetry(
            1 if st["mode"] == "control" else 0,
            st["failsafe_stage"],
            st["emergency"],
            self._fps,
            st["trim"],
            st["heartbeat_age"],
        )

        try:
            self.sock.sendto(msg, (self._vita_ip, self.telemetry_port))
        except OSError:
            pass

    # ------------------------------------------------------------ loop
    def run(self):
        self._running = True

        while self._running:
            try:
                data, addr = self.sock.recvfrom(256)
                self.handle_packet(data, addr)
            except socket.timeout:
                pass
            except ConnectionResetError:
                # Windows can report ICMP errors for a closed Vita port.
                pass
            except OSError:
                break

            self.send_telemetry()

    def stop(self):
        self._running = False

    def close(self):
        self.stop()
        try:
            self.sock.close()
        except OSError:
            pass


class TcpSlamFrameSender:
    """Forward camera frames from Windows to the WSL SLAM receiver."""

    def __init__(self, link, host, port=5011):
        self.link = link
        self.host = host
        self.port = port
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            name="tcp-slam-frame-sender",
            daemon=True,
        )
        self.sock = None
        self.last_seq = -1
        self.frames_sent = 0

    def start(self):
        self.thread.start()

    def _run(self):
        try:
            while not self.stop_event.is_set():
                try:
                    print(
                        f"[SLAM-REMOTE] Connecting to "
                        f"{self.host}:{self.port}...",
                        flush=True,
                    )
                    self.sock = socket.create_connection(
                        (self.host, self.port), timeout=5
                    )
                    self.sock.settimeout(None)
                    print(
                        "[SLAM-REMOTE] Connected to WSL receiver.",
                        flush=True,
                    )
                    break
                except OSError as exc:
                    if self.stop_event.is_set():
                        return
                    print(
                        f"[SLAM-REMOTE] Connection failed: {exc}; "
                        "retrying in 2 seconds.",
                        flush=True,
                    )
                    self.stop_event.wait(2)

            if self.sock is None or self.stop_event.is_set():
                return

            while not self.stop_event.is_set():
                frame = self.link.get_frame()

                if frame is None or frame.seq == self.last_seq:
                    self.stop_event.wait(0.005)
                    continue

                self.last_seq = frame.seq
                jpeg = frame.jpeg

                if not jpeg or len(jpeg) > 2_000_000:
                    print(
                        f"[SLAM-REMOTE] Skipping invalid JPEG "
                        f"length: {len(jpeg) if jpeg else 0}",
                        flush=True,
                    )
                    continue

                header = struct.pack("=dI", frame.ts, len(jpeg))
                self.sock.sendall(header + jpeg)
                self.frames_sent += 1

                if self.frames_sent % 30 == 0:
                    print(
                        f"[SLAM-REMOTE] Sent {self.frames_sent} frames",
                        flush=True,
                    )

        except (OSError, ConnectionError) as exc:
            if not self.stop_event.is_set():
                print(
                    f"[SLAM-REMOTE] TCP stream ended: {exc}",
                    flush=True,
                )
        finally:
            if self.sock is not None:
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                try:
                    self.sock.close()
                except OSError:
                    pass
                self.sock = None

            print(
                f"[SLAM-REMOTE] Sender stopped; "
                f"sent {self.frames_sent} frames.",
                flush=True,
            )

    def stop(self):
        self.stop_event.set()
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self.sock.close()
            except OSError:
                pass
        self.thread.join(timeout=3.0)

class SlamFrameSender:
    """Forward only the newest camera frame to the SLAM worker."""

    def __init__(self, link, process):
        self.link = link
        self.process = process
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            name="slam-frame-sender",
            daemon=True,
        )
        self.last_seq = -1
        self.frames_sent = 0

    def start(self):
        self.thread.start()

    def _run(self):
        try:
            while not self.stop_event.is_set():
                if self.process.poll() is not None:
                    print(
                        f"[SLAM] Worker exited with code "
                        f"{self.process.returncode}; stopping frame sender."
                    )
                    break

                frame = self.link.get_frame()

                if frame is None or frame.seq == self.last_seq:
                    self.stop_event.wait(0.005)
                    continue

                self.last_seq = frame.seq
                jpeg = frame.jpeg

                # Protocol expected by live_slam.cpp:
                # native-endian double timestamp + uint32 length + JPEG.
                packet_header = struct.pack("=dI", frame.ts, len(jpeg))

                try:
                    self.process.stdin.write(packet_header)
                    self.process.stdin.write(jpeg)
                    self.frames_sent += 1
                except (BrokenPipeError, OSError, ValueError) as exc:
                    if not self.stop_event.is_set():
                        print(f"[SLAM] Frame pipe closed: {exc}")
                    break

        finally:
            print(f"[SLAM] Frame sender stopped; sent {self.frames_sent} frames.")

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=3.0)


def start_slam(project_root, pose_port):
    """Start the ORB-SLAM3 worker and return (process, output_prefix)."""
    worker = project_root / "build" / "slam" / "live_slam"
    vocabulary = Path.home() / "ORB_SLAM3" / "Vocabulary" / "ORBvoc.txt"
    settings = project_root / "drone_orbslam3.yaml"

    for path in (worker, vocabulary, settings):
        if not path.is_file():
            raise FileNotFoundError(f"Required SLAM file not found: {path}")

    if not os.access(worker, os.X_OK):
        raise PermissionError(f"SLAM worker is not executable: {worker}")

    output_dir = project_root / "trajectories"
    output_dir.mkdir(parents=True, exist_ok=True)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    prefix = output_dir / f"live_{stamp}"

    process = subprocess.Popen(
        [
            str(worker),
            str(vocabulary),
            str(settings),
            str(prefix),
            str(pose_port),
        ],
        stdin=subprocess.PIPE,
        bufsize=0,
    )

    print(f"[SLAM] Worker started (PID {process.pid}).")
    print(f"[SLAM] Trajectory prefix: {prefix}")
    print(f"[SLAM] Live pose UDP destination: 127.0.0.1:{pose_port}")
    print("[SLAM] Pose stream sends JSON datagrams; tracked poses are saved to *_tracked.txt.")

    return process, prefix


def stop_slam(process, sender):
    """Stop frame forwarding and ask the worker to save its output."""
    if sender:
        sender.stop()

    if process is None:
        return

    # EOF tells live_slam.cpp to finish and call Shutdown()/export.
    try:
        if process.stdin and not process.stdin.closed:
            process.stdin.close()
    except (BrokenPipeError, OSError, ValueError):
        pass

    try:
        code = process.wait(timeout=30)
        print(f"[SLAM] Worker exited with code {code}.")
        if code != 0:
            print(
                "[SLAM] Non-zero exit detected. The tracked pose file may "
                "still contain poses; check the worker's shutdown messages."
            )
    except subprocess.TimeoutExpired:
        print("[SLAM] Worker did not exit within 30 seconds; terminating.")
        process.terminate()

        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print("[SLAM] Termination timed out; killing worker.")
            process.kill()
            process.wait()


def main():
    from drone import DroneLink
    from teleop import TRIM_FILE, load_trim, save_trim

    ap = argparse.ArgumentParser(
        description="Vita controller bridge for the WIFI_CAM drone"
    )
    ap.add_argument("--ip", default="192.168.4.153", help="drone address")
    ap.add_argument(
        "--port",
        type=int,
        default=vl.DEFAULT_PORT,
        help="UDP port the Vita sends to",
    )
    ap.add_argument("--fmt", type=int, choices=[8, 20], default=20)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--trim-file", default=TRIM_FILE)
    ap.add_argument("--land-after", type=float, default=1.5)
    ap.add_argument("--cut-after", type=float, default=5.0)
    ap.add_argument(
        "--no-vita-video",
        action="store_true",
        help="do not send the camera picture to the Vita",
    )
    ap.add_argument(
        "--vita-size",
        default="320x240",
        help="picture size sent to the Vita, WIDTHxHEIGHT (max 640x480)",
    )
    ap.add_argument(
        "--vita-fps",
        type=float,
        default=8.0,
        help="most frames per second sent to the Vita",
    )
    ap.add_argument(
        "--record",
        action="store_true",
        help="save every camera frame to recordings/<timestamp>/ (for SLAM)",
    )
    ap.add_argument(
        "--record-dir",
        default=None,
        help="folder for --record (default recordings/<timestamp>)",
    )
    ap.add_argument(
        "--slam",
        action="store_true",
        help="run live ORB-SLAM3 using the drone camera",
    )
    ap.add_argument(
        "--slam-pose-port",
        type=int,
        default=5010,
        help="local UDP port for SLAM pose output",
    )
    ap.add_argument(
        "--slam-remote",
        action="store_true",
        help="send camera frames to an ORB-SLAM3 worker running in WSL",
    )
    ap.add_argument(
        "--slam-host",
        default=None,
        help="WSL IP address for remote SLAM (required with --slam-remote)",
    )
    ap.add_argument(
        "--slam-port",
        type=int,
        default=5011,
        help="TCP port of the WSL SLAM receiver",
    )
    args = ap.parse_args()

    if args.slam and args.slam_remote:
        ap.error("Use either --slam or --slam-remote, not both")

    if args.slam_remote and not args.slam_host:
        ap.error("--slam-remote requires --slam-host")

    if args.slam_remote and args.no_video:
        ap.error("--slam-remote needs the video; remove --no-video")

    if not 1 <= args.slam_port <= 65535:
        ap.error("--slam-port must be between 1 and 65535")

    if args.record and args.no_video:
        ap.error("--record needs the video; remove --no-video")

    if args.slam and args.no_video:
        ap.error("--slam needs the video; remove --no-video")

    if not 1 <= args.slam_pose_port <= 65535:
        ap.error("--slam-pose-port must be between 1 and 65535")

    try:
        width, height = (
            int(v) for v in args.vita_size.lower().split("x")
        )
        if not (16 <= width <= 640 and 16 <= height <= 480):
            raise ValueError
    except ValueError:
        ap.error(
            "--vita-size must look like 320x240 "
            "(width 16-640, height 16-480)"
        )

    # Preserve the original safety interlock.
    ans = input(
        "Are the PROPS REMOVED (or the drone tethered / in a cage)? Type YES: "
    )
    if ans.strip() != "YES":
        print("Aborted.")
        return

    link = DroneLink(
        ip=args.ip,
        fmt=args.fmt,
        video=not args.no_video,
        land_after=args.land_after,
        cut_after=args.cut_after,
    )
    link.set_trim(**load_trim(args.trim_file))

    bridge = VitaBridge(
        link,
        port=args.port,
        on_trim_change=lambda t: save_trim(t, args.trim_file),
    )

    forwarder = None
    recorder = None
    slam_process = None
    slam_sender = None
    remote_slam_sender = None
    print(
        f"Bridge listening on UDP {args.port}. "
        "Start the Vita app and point it at this PC's address."
    )

    try:
        link.start()

        if args.slam:
            project_root = Path(__file__).resolve().parent.parent
            slam_process, _ = start_slam(
                project_root,
                args.slam_pose_port,
            )
            slam_sender = SlamFrameSender(link, slam_process)
            slam_sender.start()


        if args.slam_remote:
            remote_slam_sender = TcpSlamFrameSender(
                link,
                args.slam_host,
                args.slam_port,
            )
            remote_slam_sender.start()
            print(
                f"[SLAM-REMOTE] Sending frames to "
                f"{args.slam_host}:{args.slam_port}. "
                "Press Ctrl+C to stop.",
                flush=True,
            )


        if not args.no_vita_video and not args.no_video:
            from videoforward import VideoForwarder

            forwarder = VideoForwarder(
                link,
                lambda: (
                    (bridge.vita_ip, vl.VITA_PORT)
                    if bridge.vita_ip
                    else None
                ),
                size=(width, height),
                fps=args.vita_fps,
            )
            forwarder.start()
            print(
                f"Sending camera picture to the Vita at {width}x{height}, "
                f"up to {args.vita_fps:g} fps."
            )

        if args.record:
            from recorder import FrameRecorder

            record_dir = args.record_dir or os.path.join(
                "recordings", time.strftime("%Y%m%d_%H%M%S")
            )
            recorder = FrameRecorder(link, record_dir)
            recorder.start()
            print(f"Recording camera frames to {record_dir}")

        if args.slam:
            print("[SLAM] Live mode enabled. Press Ctrl+C to stop.")

        bridge.run()

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        print("Stopping motors and closing...")

        # Stop optional data consumers before closing the drone link.
        if recorder:
            try:
                recorder.stop()
            except Exception as exc:
                print(f"[bridge] Recorder shutdown error: {exc}")

        if forwarder:
            try:
                forwarder.stop()
            except Exception as exc:
                print(f"[bridge] Video forwarder shutdown error: {exc}")


        if remote_slam_sender:
            try:
                remote_slam_sender.stop()
            except Exception as exc:
                print(f"[SLAM-REMOTE] Shutdown error: {exc}")

        if slam_process:
            try:
                stop_slam(slam_process, slam_sender)
            except Exception as exc:
                print(f"[SLAM] Shutdown error: {exc}")

        try:
            save_trim(link.get_trim(), args.trim_file)
        except Exception as exc:
            print(f"[bridge] Could not save trim: {exc}")

        try:
            bridge.close()
        except Exception as exc:
            print(f"[bridge] Bridge shutdown error: {exc}")

        try:
            link.close()
        except Exception as exc:
            print(f"[bridge] Drone link shutdown error: {exc}")

        print("Done.")


if __name__ == "__main__":
    main()
