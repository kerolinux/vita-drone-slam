"""Offline landing regressions: never start a network link or send a flight packet."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import autonomous_land as landing
from drone import DroneLink


class LandingTests(unittest.TestCase):
    def aligned(self):
        controller = landing.Lander(0)
        for i in range(10):
            controller.update((0, 0, 0.5), i * 0.1)
        self.assertEqual(controller.state, "DESCEND")
        return controller

    def test_off_centre_low_tag_never_triggers_landing(self):
        controller = self.aligned()
        state, cmd = controller.update((0.8, 0.8, 0.2), 1)
        self.assertEqual(state, "ALIGN")
        self.assertEqual(cmd["throttle"], 0)
        self.assertNotEqual(cmd["roll"], 0)

    def test_stable_centred_low_tag_can_land(self):
        controller = self.aligned()
        state, cmd = controller.update((0, 0, 0.2), 1)
        self.assertEqual(state, "LAND")
        self.assertEqual(cmd, landing.NEUTRAL)

    def test_one_centred_frame_or_fast_crossing_cannot_descend(self):
        controller = landing.Lander(0)
        self.assertEqual(controller.update((0, 0, 0.2), 0)[0], "ALIGN")
        for i in range(1, 20):
            state, cmd = controller.update(((-1) ** i * 0.1, 0, 0.2), i * 0.1)
            self.assertEqual(state, "ALIGN")
            self.assertEqual(cmd["throttle"], 0)

    def test_gap_resets_alignment_and_derivative(self):
        controller = self.aligned()
        state, cmd = controller.update((0.05, 0, 0.5), 2)
        self.assertEqual(state, "ALIGN")
        self.assertAlmostEqual(cmd["roll"], 0.05)

    def test_loss_stops_descent_then_requests_landing_without_yaw(self):
        controller = self.aligned()
        state, cmd = controller.update(None, 1.0)
        self.assertEqual(state, "ALIGN")
        self.assertEqual(cmd, landing.NEUTRAL)
        self.assertEqual(controller.update(None, 2.5)[0], "LAND")

    def test_small_error_survives_wire_quantisation_and_limits_hold(self):
        link = DroneLink(max_offset=landing.SPEED_PRESETS[1])
        for error in (-0.05, 0.05):
            value = landing.lateral_command(error, 0, 1, 20)
            self.assertNotEqual(link._stick_byte("roll", value), 128)
            self.assertLessEqual(abs(value), landing.MAX_XY)
        self.assertEqual(landing.lateral_command(0.01, 0, 1, 20), 0)
        self.assertEqual(landing.lateral_command(10, 10, 1, 20), landing.MAX_XY)

    def test_signs_invert_only_selected_axis(self):
        a = landing.Lander(0, 1, -1).update((0.5, 0.5, 1), 0)[1]
        b = landing.Lander(0, -1, 1).update((0.5, 0.5, 1), 0)[1]
        self.assertEqual(a["roll"], -b["roll"])
        self.assertEqual(a["pitch"], -b["pitch"])

    def test_age_uses_arrival_timestamp_and_rejects_clock_reversal(self):
        frame = SimpleNamespace(ts=100)
        self.assertEqual(landing.frame_age(frame, 102), 2)
        self.assertEqual(landing.frame_age(frame, 99), float("inf"))
        self.assertEqual(landing.frame_age(None, 102), float("inf"))

    def test_preview_blocks_all_control_paths(self):
        link = landing.PreviewLink(log=lambda *_: None)
        link._sock = Mock()
        link._mode = "control"
        link.takeoff()
        link.land()
        link.emergency_stop()
        self.assertEqual(link._next_packets(10**20), [])
        with self.assertRaises(RuntimeError):
            link._send(b"control", link.control_port)
        link._sock.sendto.assert_not_called()
        link._send(b"Bv", link.video_port)
        link._sock.sendto.assert_called_once_with(b"Bv", (link.ip, link.video_port))

    def test_keys_do_not_send_emergency_in_preview(self):
        link = Mock()
        self.assertEqual(landing.handle_key(ord("x"), link, False), "STOP")
        link.emergency_stop.assert_not_called()
        self.assertEqual(landing.handle_key(ord("x"), link, True), "STOP")
        link.emergency_stop.assert_called_once()
        self.assertEqual(landing.handle_key(ord("q"), link, True), "LAND")

    def test_land_rejection_is_retried_and_not_treated_as_success(self):
        sequence = landing.LandingSequence()
        link = Mock()
        link.land.side_effect = [False, True]
        self.assertFalse(sequence.step(link, 0))
        self.assertIsNone(sequence.accepted_at)
        self.assertFalse(sequence.step(link, 0.2))
        self.assertFalse(sequence.step(link, 1))
        self.assertTrue(sequence.step(link, 3.3))
        self.assertEqual(link.land.call_count, 2)

    def test_corrected_optical_centre_and_known_height(self):
        K = np.array([[200., 0, 110], [0, 200., 155], [0, 0, 1.]])
        points = np.array([[[90., 135], [130, 135], [130, 175], [90, 175]]])
        detector = Mock()
        detector.detectMarkers.return_value = ([points], np.array([[0]]), [])
        obs, centre = landing.find_tag(np.zeros((320, 240, 3), np.uint8), detector, K, np.zeros(5))
        self.assertTrue(np.allclose(obs, [0, 0, 0.5]))
        detector.detectMarkers.assert_called_once()

    def test_invalid_observation_is_not_used(self):
        state, cmd = landing.Lander(0).update((float("nan"), 0, 0.1), 0.1)
        self.assertEqual(state, "ALIGN")
        self.assertEqual(cmd, landing.NEUTRAL)




class MainLoopTests(unittest.TestCase):
    def run_fake(self, keys, stale=False, fail_after_takeoff=False, preview=False, vita=False, manual_roll=0.5, missing_tag=False):
        import itertools
        import tempfile
        from contextlib import ExitStack
        from pathlib import Path
        from vita_takeover import VitaTakeover as RealTakeover
        owner = []
        class LocalTakeover(RealTakeover):
            def __init__(self, raw, **kwargs):
                super().__init__(raw, port=0, bind="127.0.0.1", preview=kwargs.get("preview", False), log=lambda *_: None)
                owner.append(self)
            def start(self):
                self.raw.start()
                self.receive_sticks(dict(roll=0., pitch=0., throttle=0., yaw=0.))
        link = Mock()
        link.takeoff.return_value = True
        link.land.return_value = True
        link.get_trim.return_value = {"roll": -2, "pitch": -3, "yaw": -1}
        link._stick_byte.return_value = 128
        sequence, clock, detections = itertools.count(), itertools.count(), itertools.count()
        def frame():
            if vita and owner:
                owner[0].receive_sticks(dict(roll=manual_roll if link.takeoff.called else 0., pitch=0., throttle=0., yaw=0.))
            if fail_after_takeoff and link.takeoff.called:
                raise RuntimeError("synthetic decode failure")
            return SimpleNamespace(jpeg=b"x", seq=next(sequence),
                                   ts=landing.time.time() - (2 if stale else 0))
        link.get_frame.side_effect = frame
        key_stream = iter(keys)
        with tempfile.TemporaryDirectory() as folder, ExitStack() as stack:
            if vita:
                stack.enter_context(patch.object(landing, "VitaTakeover", LocalTakeover))
                stack.enter_context(patch.object(landing, "LAND_WAIT_S", 0.03))
            stack.enter_context(patch.object(landing, "PreviewLink" if preview else "DroneLink", return_value=link))
            stack.enter_context(patch.object(landing, "ensure_bottom_camera", return_value=True))
            stack.enter_context(patch.object(landing.vision, "load_calibration",
                                return_value=(np.eye(3), np.zeros(5), (240, 320))))
            stack.enter_context(patch.object(landing.vision, "make_detector", return_value=Mock()))
            stack.enter_context(patch.object(landing, "find_tag", side_effect=lambda *_:
                                None if missing_tag or (preview and next(detections) < 4) else
                                ((0.5 if preview else 0., 0., 0.5), np.array([100., 100.]))))
            stack.enter_context(patch.object(landing, "read_key", side_effect=lambda: next(key_stream, -1)))
            stack.enter_context(patch.object(landing.time, "monotonic", side_effect=lambda: next(clock) * (0.01 if vita else 0.2)))
            stack.enter_context(patch.object(landing.time, "sleep"))
            for name in ("namedWindow", "imshow", "destroyAllWindows"):
                stack.enter_context(patch.object(landing.cv2, name))
            stack.enter_context(patch.object(landing.cv2, "imdecode", return_value=np.zeros((320, 240, 3), np.uint8)))
            landing.main(([] if preview else ["--live"]) + (["--vita-takeover"] if vita else []) + ["--log", str(Path(folder) / "test.csv")])
            self.assertTrue((Path(folder) / "test.csv").exists())
            import csv
            with (Path(folder) / "test.csv").open() as f:
                link.rows = list(csv.DictReader(f))
        link.close.assert_called_once_with(stop_motors=False)
        return link

    def test_preview_recovers_after_initial_tag_loss_without_flight_commands(self):
        link = self.run_fake([-1] * 12 + [ord("x")], preview=True)
        link.takeoff.assert_not_called()
        link.land.assert_not_called()
        link.set_sticks.assert_not_called()
        link.emergency_stop.assert_not_called()
        self.assertTrue(any(float(row["roll"]) > 0 for row in link.rows))

    def test_vita_handoff_in_main_loop_keeps_manual_commands_and_q_works(self):
        link = self.run_fake([ord("t"), -1, -1, ord("q")], vita=True)
        manual = [r for r in link.rows if r["phase"] == "MANUAL"]
        self.assertTrue(manual)
        self.assertEqual(float(manual[0]["roll"]), 0.)  # manual takeoff starts centred
        self.assertTrue(all(float(r["roll"]) == 0.5 for r in manual[1:]))
        link.land.assert_called_once()
        link.takeoff.assert_called_once()

    def test_vita_manual_takeoff_needs_no_tag_and_no_tag_cannot_engage(self):
        link = self.run_fake([ord("t"), ord("a"), -1, ord("q")], vita=True,
                             manual_roll=0, missing_tag=True)
        link.takeoff.assert_called_once()
        self.assertTrue(all(r["phase"] == "MANUAL" for r in link.rows if r["phase"] != "LAND"))

    def test_vita_explicit_engagement_with_fresh_tag(self):
        link = self.run_fake([ord("t"), ord("a"), -1, ord("q")], vita=True, manual_roll=0)
        self.assertEqual(link.rows[0]["phase"], "MANUAL")
        self.assertTrue(any(r["phase"] == "ALIGN" for r in link.rows))

    def test_vita_stale_video_cannot_engage_after_manual_takeoff(self):
        link = self.run_fake([ord("t"), ord("a"), ord("q")], vita=True, manual_roll=0, stale=True)
        link.takeoff.assert_called_once()
        self.assertFalse(any(r["phase"] == "ALIGN" for r in link.rows))

    def test_q_during_takeoff_queues_land(self):
        link = self.run_fake([ord("t"), ord("q")])
        link.takeoff.assert_called_once()
        link.land.assert_called_once()
        link.emergency_stop.assert_not_called()
        link.set_trim.assert_called_once_with(roll=-2, pitch=-3, yaw=-1)

    def test_x_during_takeoff_sends_emergency(self):
        link = self.run_fake([ord("t"), ord("x")])
        link.emergency_stop.assert_called_once()
        link.land.assert_not_called()

    def test_stale_frame_prevents_takeoff(self):
        link = self.run_fake([ord("t"), ord("q")], stale=True)
        link.takeoff.assert_not_called()
        link.land.assert_not_called()

    def test_exception_after_takeoff_requests_land_before_close(self):
        link = self.run_fake([ord("t")], fail_after_takeoff=True)
        link.land.assert_called_once()


if __name__ == "__main__":
    unittest.main()
