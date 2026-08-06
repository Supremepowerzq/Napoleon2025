# Bronchoscopy Position Recognition PyQt Console

This folder is the standalone English interface for the existing
`Bronchialtree_identification` stack. It places the bronchial position map and
the live bronchoscope feed in one PyQt5 window while preserving the Chinese
version's robot, motor-limit, controller, detection, recording, and AutoNav
behavior.

## Main interface

- Left: anatomical model, confirmed position, traversed path, and candidate position.
- Right: a clean live bronchoscope view without boxes, class labels, or confidence text.
- Header: current position, F/B/S motion, robot mode, controller, and camera status.
- Footer: object detection, recording, manual mode, stop, return to zero, set zero, and power off.
- Menus: autonomous inspection, individual field-recorded AutoNav paths, pause/resume, and stop.

When object detection is off, the camera and recording remain available but
YOLO inference is skipped. After detection is enabled, the same confirmed
position must appear in four consecutive inference updates at confidence 0.60
or higher before the map is illuminated. A low-confidence result, a missed
detection, or a position change restarts that counter.

The map coordinates are registered to the visible airway centreline in
`assets/bronchi.png`. Topology edges pass through the anatomical carina and
lobar take-offs instead of drawing straight chords across the image.

Recordings are saved to:

```text
Bronchialtree_identification/recordings/
```

## Launch

From `G:\zq\Napoleon2025\2026`:

```powershell
C:\Users\lenovo\.conda\envs\Napoleon_2026\python.exe -m Bronchialtree_identification.recognition_gui_en.launcher
```

The convenience entry point is also available:

```powershell
C:\Users\lenovo\.conda\envs\Napoleon_2026\python.exe Bronchialtree_identification\run_recognition_gui_en.py
```

## Hardware-free verification

```powershell
C:\Users\lenovo\.conda\envs\Napoleon_2026\python.exe -m Bronchialtree_identification.recognition_gui_en.launcher --check-only
$env:QT_QPA_PLATFORM='offscreen'
C:\Users\lenovo\.conda\envs\Napoleon_2026\python.exe -m Bronchialtree_identification.recognition_gui_en.launcher --ui-smoke-test
C:\Users\lenovo\.conda\envs\Napoleon_2026\python.exe -m unittest Bronchialtree_identification.recognition_gui_en.test_gui_logic -v
```

## Safety boundaries

- `video_worker.py` owns the camera, YOLO inference, and recording only; it does not access serial ports or issue motor commands.
- `robot_runtime.py` loads the existing AutoNav application and uses its established UI command queue.
- The motor-feedback adapter only reads existing cached feedback and does not open another serial connection.
- Inspection uses the latest single RMB path in `AutoNavdatasets`; it does not average paths.
- The TR menu item uses the original three-axis fixed-origin return routine.
- Layers 2 and 3 remain disabled by default for path navigation from this interface.
