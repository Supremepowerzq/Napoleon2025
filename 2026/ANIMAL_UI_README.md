# Animal Experiment UI

Independent launcher: `animal_experiment_ui.py`.
Place it beside `main2026-Xhandwriting-AC-AutoNav(Animal_test)-Bronchus.py`.
The original source is read unchanged at launch. No copied control logic is maintained.

## Start

In the same Python/Conda environment used for the original experiment program:

```bat
cd /d G:\zq\Napoleon2025\2026
python animal_experiment_ui.py
```

UI-only preview (does not import the controller dependencies or start hardware):

```bat
D:\ProgramData\anaconda3\python.exe animal_experiment_ui.py --preview
```

Automated UI checks, with automatic close:

```bat
D:\ProgramData\anaconda3\python.exe animal_experiment_ui.py --preview --verify
```

## Presentation changes

English controls, status labels and menus; blue / pale green / gray gradient header and buttons.
Manual controls and mode controls sit side by side. All sections remain scrollable.
Stop and power-off controls retain distinct text and stronger borders.
All original command tokens, motor callbacks, selection values, filename values, recording events,
planning validation and navigation arguments remain in the original controller.
The new presentation facade applies only inside this new process.
Chinese voice commands and recognition settings remain unchanged, as part of the original functionality.
Original backend console messages remain unchanged. Any diagnostic not covered by the English
presentation dictionary is retained verbatim in `animal-ui-original-messages.log`; the English UI
refers to that file and keeps the diagnostic's ASCII data.

## Verification limits

The UI-only checks exercised command forwarding, raw selection preservation, navigation arguments,
state/recording/progress updates and English labels on the workstation. The original source SHA-256:
`dccee4c34203b866b0de2bb14de2faa20b344a282f193d6f67079bfdf426f380`.
No animal experiment, motor motion or camera inference was started for validation.
SSH desktop screenshots were unavailable; final visual review should use --preview on the workstation.
