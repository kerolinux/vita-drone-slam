# Autonomous landing test procedure

Run from the repository root with the existing virtual environment activated.

## 1. Camera and command preview first

Remove the propellers, connect to the drone Wi-Fi, and stop other apps/scripts controlling the drone.

```powershell
python python/autonomous_land.py
```

This mode sends only video start/stop/camera-toggle packets. Flight-control packets are blocked at the
transport, including emergency/failsafe packets. The console shows computed commands, actual stick bytes
including trim, and frame arrival age. CSV diagnostics go to `logs/landing_<time>.csv`.

Check:
- The bottom camera is selected and the tag is detected.
- The tag's black-square side is really 100 mm, matching `TAG_SIZE_M`. Do not confuse calibration checkerboard
  square size (25 mm) with AprilTag size.
- At several known distances, the displayed/logged distance is plausible. Height is still an apparent-size
  estimate, assumes a near-frontoparallel tag, and is not an altimeter.
- Move the tag to each side of the camera while keeping the drone stationary; observe command signs and
  wire bytes. This checks software/image mapping, NOT the real aircraft's response.
- Check saved trim printed at startup. The script now loads the repository's `trim.json`; these old values
  are not automatically known to be correct for the current battery/airframe.
- Q/Esc/X exit preview without sending flight commands.

The preview simulates landing decisions and starts a new simulated trial when it sees a tag again
after reaching LAND. A preview or unit-test pass does not establish safe flight.

## 2. Resolve physical uncertainties before autonomous flight

Verify manual hover/drift, roll/pitch responses, and the one-key landing behaviour separately using the
existing control setup and a clear test area, not beside walls. Do not increase gains or invert signs by
trial during an autonomous descent. A propellers-off preview cannot establish actual flight directions.

The default signs remain roll +1 and pitch -1 and are UNVERIFIED. Override with `--roll-sign` and
`--pitch-sign` only after measuring the correct mapping. Neutral sticks (including trim) are not a
position-hold guarantee. No code here measures horizontal vehicle velocity.

## 3. Standalone live controls without Vita (only after the checks above)

```powershell
python python/autonomous_land.py --live
```

- Starts in READY. It no longer takes off automatically.
- Focus the OpenCV video window. Press T only when ready; takeoff is rejected without a fresh detected tag.
- Q/Esc request landing, including during takeoff. Before takeoff they simply exit.
- X requests an emergency motor cut; this can cause a fall.
- Missing/invalid/stale observations stop corrections/descent; after 1.5 seconds without an accepted target,
  the script requests landing in place. There is no blind yaw search.
- Alignment must remain within tolerance with low image-error rate for 0.6 seconds before descent.
- Drifting out of alignment returns to ALIGN and stops descent. Low estimated height alone cannot trigger LAND.
- Landing commands rejected while another flag pulse is pending are retried. Queued commands are not
  acknowledgement from the drone. After the pulse wait, the script exits; visually confirm touchdown.

Firmware owns the final one-key landing. Lateral overrides during that firmware mode are deliberately
not added without protocol evidence. The shared 20-byte takeoff/land bit is unchanged.

## Optional PS Vita takeover

Stop any separate `bridge.py`, teleop script, or other drone controller. This option reuses the
existing Vita app and runs its receiver inside the landing process, sharing one drone connection.
Configure the Vita app to send to the PC's reachable IP, UDP port 9000 (override with `--vita-port`).
Telemetry is returned to the Vita; this mode does not forward video to its screen.

First test with propellers removed:

```powershell
python python/autonomous_land.py --vita-takeover
```

Move a stick at least 18% from centre and check that the PC displays MANUAL. Centre the sticks and
confirm it stays MANUAL. Cross requests LAND; Circle stops the run. Preview still blocks all flight
packets. Restart the script after this test.

After completing the physical checks above, enable flight with:

```powershell
python python/autonomous_land.py --live --vita-takeover
```

- Start with the drone on the ground. The bottom-camera stream must open, but no tag is required for takeoff.
- Hold Vita Triangle (the app's existing takeoff gesture) or press PC T to take off manually. A connected
  Vita with centred sticks is required. Fly above the tag using the Vita; missing tags do not trigger
  automatic landing while you are in MANUAL.
- When the tag is visible and the drone is stable, centre the Vita sticks, focus the PC video window,
  and press A to engage autonomous alignment/descent/landing. Missing or stale tags, a stale controller,
  deflected sticks, or a terminal LAND/EMERGENCY state prevent engagement.
- A is an explicit handoff, not proof of stable hover. Confirm physical control directions before engaging.
- Moving either Vita stick at least 18% latches manual control, including during takeoff. There is no
  automatic return to autonomy. You may explicitly press A again with a fresh tag and centred sticks.
  Once landing has been requested, restart only after confirming touchdown.
- Right stick: roll/pitch. Left stick: throttle/yaw. START selects the existing manual speed presets.
- Cross: request landing. Circle: emergency motor cut, which can cause a fall. PC Q/Esc and X remain available.
- Repeated takeoff is blocked after the first accepted takeoff. Calibration and trim buttons are disabled; saved trim is preserved.
- If valid Vita packets stop, commands go neutral after 0.3 seconds and landing is requested after
  1.5 seconds, in both autonomous and manual flight. Neutral does not guarantee position hold.
- The receiver and timeout checks run independently of camera processing. The first valid Vita IP
  is retained for the session; restart if you need to change controllers.

Take over early: stick takeover cannot cancel a firmware landing request already issued. The Vita
shares the PC and drone radio connection, so it is not an independent emergency radio and cannot
recover from every PC, process, or radio failure. Physical takeover and flight response remain unverified.

## Offline regressions

These do not contact the physical drone. The driver suite uses a fake drone on localhost.

```powershell
python -B -m unittest discover -s python -p test_autonomous_land.py -v
python -B -m unittest discover -s python -p test_drone.py -v
python -B -m unittest discover -s python -p test_vita_takeover.py -v
python -B -m unittest discover -s python -p test_bridge.py -v
```

No sensors provide touchdown confirmation. Arrival age cannot detect delay accumulated before a frame
reaches the PC. Keep the CSV from each test; do not interpret preview command bytes as measured thrust.

